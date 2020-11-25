/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * Test code for the SystemIdentification class
 * Run this test only using make tests TESTFILTER=system_identification
 */

#include <gtest/gtest.h>
#include <matrix/matrix/math.hpp>

#include "system_identification.hpp"

using namespace matrix;

class SystemIdentificationTest : public ::testing::Test
{
public:
	SystemIdentificationTest() {};
};

TEST_F(SystemIdentificationTest, basicTest)
{
	constexpr float fs = 800.f;

	SystemIdentification _sys_id;
	_sys_id.setHpfCutoffFrequency(fs, 0.05f);
	_sys_id.setLpfCutoffFrequency(fs, 30.f);
	_sys_id.setForgettingFactor(80.f, 1.f / fs);

	for (int i = 0; i < 10; i += 2) {
		_sys_id.update(float(i), float(i + 1));
	}

	const Vector<float, 5> coefficients = _sys_id.getCoefficients();
	float data_check[] = {-2.51f, 2.39f, 12.21f, -8.44f, -10.28f};
	const Vector<float, 5> coefficients_check(data_check);
	float eps = 1e-2;
	EXPECT_TRUE((coefficients - coefficients_check).abs().max() < eps);
	coefficients.print();
	_sys_id.getVariances().print();
}

TEST_F(SystemIdentificationTest, resetTest)
{
	constexpr float fs = 800.f;

	SystemIdentification _sys_id;
	_sys_id.setHpfCutoffFrequency(fs, 0.05f);
	_sys_id.setLpfCutoffFrequency(fs, 30.f);
	_sys_id.setForgettingFactor(80.f, 1.f / fs);

	for (int i = 0; i < 10; i += 2) {
		_sys_id.update(float(i), float(i + 1));
	}

	const Vector<float, 5> coefficients = _sys_id.getCoefficients();

	// WHEN: resetting
	_sys_id.reset();

	// THEN: the variances and coefficients should be properly reset
	EXPECT_TRUE(_sys_id.getCoefficients().abs().max() < 1e-8f);
	EXPECT_TRUE(_sys_id.getVariances().min() > 9e3f);

	// AND WHEN: running the same sequence of inputs-outputs
	for (int i = 0; i < 10; i += 2) {
		_sys_id.update(float(i), float(i + 1));
	}

	// THEN: the result should be exactly the same
	EXPECT_TRUE((coefficients - _sys_id.getCoefficients()).abs().max() < 1e-8f);
	coefficients.print();
	_sys_id.getVariances().print();
}
