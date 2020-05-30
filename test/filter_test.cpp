///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018, S.O. Engineering, LLC
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of S.O. Engineering, LLC nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/// \author James Holley

#include <gtest/gtest.h>

#include <bobble_controllers/Filter.h>
#include <bobble_controllers/HighPassFilter.h>
#include <bobble_controllers/LowPassFilter.h>

// The fixture for testing class Foo.
class FilterTest : public ::testing::Test {
protected:
    // You can remove any or all of the following functions if their bodies would
    // be empty.

    FilterTest() {
        // You can do set-up work for each test here.
    }

    ~FilterTest() override {
        // You can do clean-up work that doesn't throw exceptions here.
    }

    // If the constructor and destructor are not enough for setting up
    // and cleaning up each test, you can define the following methods:

    void SetUp() override {
        // Code here will be called immediately after the constructor (right
        // before each test).
    }

    void TearDown() override {
        // Code here will be called immediately after each test (right
        // before the destructor).
    }

    // Class members declared here can be used by all tests in the test suite
    // for Foo.
};

/// This test assumes a sample rate of 500, filter of 50 Hz, and damping of 1.0
/// Input is a unit step, step results were generated from octave

TEST_F(FilterTest, HPFTest) {
    HighPassFilter hpfFilter;
    hpfFilter.resetFilterParameters(0.002, 50, 1.0);
    const unsigned int RESPONSE_LENGTH = 26;
    const float ACCEPTABLE_ERROR = 1e-6;
    float response[RESPONSE_LENGTH] = {
             0.5790339089,
             0.0253449543,
            -0.1312539676,
            -0.1439021654,
            -0.1144519971,
            -0.0802678418,
            -0.0526085896,
            -0.0330492127,
            -0.0201670995,
            -0.0120484024,
            -0.0070829718,
            -0.0041114438,
            -0.0023622560,
            -0.0013458431,
            -0.0007613574,
            -0.0004281229,
            -0.0002394956,
            -0.0001333731,
            -0.0000739809,
            -0.0000408930,
            -0.0000225332,
            -0.0000123817,
            -0.0000067864,
            -0.0000037111,
            -0.0000020252,
            -0.0000011030,
    };

    double filterResponse = 0.0;
    for(int i = 0; i < RESPONSE_LENGTH; i++) {
        filterResponse = hpfFilter.filter(1.0);
        ASSERT_NEAR(filterResponse, response[i], ACCEPTABLE_ERROR);
    }
}

/// This test assumes a sample rate of 500, filter of 50 Hz, and damping of 1.0
/// Input is a unit step, step results were generated from octave
TEST_F(FilterTest, LPFTest) {
    LowPassFilter lpfFilter;
    lpfFilter.resetFilterParameters(0.002, 50, 1.0);
    const unsigned int RESPONSE_LENGTH = 26;
    const float ACCEPTABLE_ERROR = 1e-6;
    float response[RESPONSE_LENGTH] = {
            0.057148,
            0.231095,
            0.454238,
            0.639772,
            0.772651,
            0.860813,
            0.916642,
            0.950903,
            0.971458,
            0.983581,
            0.990636,
            0.994698,
            0.997016,
            0.998330,
            0.999069,
            0.999484,
            0.999714,
            0.999843,
            0.999913,
            0.999953,
            0.999974,
            0.999986,
            0.999992,
            0.999996,
            0.999998,
            0.999999,
    };

    double filterResponse = 0.0;
    for (int i = 0; i < RESPONSE_LENGTH; i++) {
        filterResponse = lpfFilter.filter(1.0);
        ASSERT_NEAR(filterResponse, response[i], ACCEPTABLE_ERROR);
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
