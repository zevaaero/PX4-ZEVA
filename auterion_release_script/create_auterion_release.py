from dockerfile_parse import DockerfileParser
from git import Repo
import re
import shutil
import hashlib
import subprocess


class GitNode:
    def __init__(self, repo, repo_name):
        self.dependencies = []
        self.repo = repo
        self.repo_name = repo_name

    def add_dependency(self, git_node):
        self.dependencies.append(git_node)


class CreateRelease:
    _default_url = "git@github.com:Auterion/"
    _default_build_dir = "/tmp/release/"
    _root_node = GitNode(None, "root")
    _git_nodes = {}
    _version = None

    def __init__(self, version):
        self._version = version
        try:
            shutil.rmtree(self._default_build_dir)
        except Exception as e:
            pass

        print("Preparing release " + version)

    def update_dockerfile_repo_checksum(self, distro_repo_name, repo_name, dockerfile_name):
        git_hash = self._git_nodes[repo_name].repo.head.object.hexsha
        dfp = DockerfileParser(cache_content=True,
                               fileobj=open(self._git_nodes[distro_repo_name].repo.working_dir + "/" + dockerfile_name,
                                            'rb+'))
        for i in dfp.structure:
            if "RUN" in i["instruction"] and "git clone" in i["content"] and repo_name in i["content"]:
                print("Updating docker instruction : \n {0}".format(i["content"]))
                new_content = re.sub(r"\bcheckout *[a-f0-9]{40}\b", "checkout %s" % git_hash, i["content"])
                dfp.add_lines_at(i, new_content, replace=True)
                print("New instruction : \n {0}".format(new_content))
                break


    def add_release_repo(self, branch, repo_name, url=_default_url):
        print("Adding repo " + repo_name)
        path = self._version + "/" + hashlib.md5((url + repo_name).encode()).hexdigest() + "-" + repo_name
        repo = Repo.clone_from(url + repo_name + ".git", self._default_build_dir + path)
        print("Checking out branch {0} from repo {1}.git".format(branch, repo_name))
        repo.git.checkout(branch)
        for submodule in repo.submodules:
            submodule.update(init=True)
        git_node = GitNode(repo, repo_name)
        self._git_nodes[repo_name] = git_node
        self._root_node.add_dependency(git_node)

    def update_repo_file(self, repo_name, src_file, to_repo_path):
        repo_full_path = self._git_nodes[repo_name].repo.working_dir + "/" + to_repo_path
        if subprocess.call(["cp", src_file, repo_full_path]):
            raise Exception("Updating file " + src_file)
        self._git_nodes[repo_name].repo.git.add(repo_full_path)
        self._git_nodes[repo_name].repo.git.commit("-m", "updated " + to_repo_path)

    def submodule_hr_name(self, submodule):
        return submodule.url.split("/")[-1]

    def create_submodule_dependency_list(self, current_node, update_order_list):
        if not current_node.dependencies:
            if current_node in update_order_list:
                update_order_list.remove(current_node)
            update_order_list.append(current_node)
            return
        for dependency in current_node.dependencies:
            self.create_submodule_dependency_list(dependency, update_order_list)
            if current_node not in update_order_list:
                update_order_list.insert(0, current_node)

    def create(self):

        print("Creating release {0}".format(self._version))

        # Create a submodule dependency tree so we can update
        # and submodules that are dependent on release changes
        # last

        for dependency in self._root_node.dependencies:
            for git_node_submodule in dependency.repo.submodules:
                sub_repo_name = self.submodule_hr_name(git_node_submodule)
                if sub_repo_name in self._git_nodes.keys():
                    dependency.add_dependency(self._git_nodes[sub_repo_name])

        push_order = []

        self.create_submodule_dependency_list(self._root_node, push_order)
        push_order.remove(self._root_node)

        for git_node in reversed(push_order):

            for submodule in git_node.repo.submodules:
                submodule_name = self.submodule_hr_name(submodule)
                print("Updating submodule " + submodule_name + " in repo " + git_node.repo_name)
                submodule_repo = submodule.module()
                if submodule_name in self._git_nodes.keys():
                    tag_sha = self._git_nodes[submodule_name].repo.head.object.hexsha
                    submodule_repo.git.fetch("origin", tag_sha)
                    submodule_repo.git.checkout(tag_sha)
                    git_node.repo.git.add(submodule.name)

            print("Creating and pushing tag {0} in repo {1} ".format(self._version, git_node.repo_name))

            if git_node.repo.is_dirty():
                git_node.repo.git.commit("-am", self._version)

            new_tag = git_node.repo.create_tag(self._version)
            git_node.repo.remotes.origin.push(new_tag)
