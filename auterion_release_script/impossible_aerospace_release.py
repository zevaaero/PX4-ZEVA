import create_auterion_release as cr

release = cr.CreateRelease("IA-v1.0.4.0-uavcan-img")

release.add_release_repo("IA-release-1",
                         "auterion_distro_resin")

release.add_release_repo("master",
                         "impossible_aero_distro")

release.add_release_repo("IA-release-1",
                         "auterion_firmware_uploader")

release.update_repo_file("auterion_firmware_uploader",
                         "/home/jimmy/Development/Auterion/Firmware/build/px4_fmu-v3_default/px4_fmu-v3_default.px4",
                         "firmware_versions/impossible_v1.9/fmuv3")

release.update_dockerfile_repo_checksum("auterion_distro_resin",
                                        "auterion_firmware_uploader",
                                        "Dockerfile.beaglebone-pocket")
release.create()

