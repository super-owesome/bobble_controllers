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

/// This test assumes a sample rate of 500, filter of 0.01 Hz, and damping of 0.707
/// Input is a unit step, step results were generated from octave

TEST_F(FilterTest, HPFTest) {
    HighPassFilter hpfFilter;
    const unsigned int RESPONSE_LENGTH = 251;
    const float ACCEPTABLE_ERROR = 1e-4;
    float response[RESPONSE_LENGTH] = {
            0.977745332,
            0.933742229,
            0.890750855,
            0.848769269,
            0.807794617,
            0.767823170,
            0.728850373,
            0.690870880,
            0.653878595,
            0.617866716,
            0.582827766,
            0.548753635,
            0.515635612,
            0.483464425,
            0.452230271,
            0.421922849,
            0.392531394,
            0.364044708,
            0.336451190,
            0.309738864,
            0.283895407,
            0.258908179,
            0.234764248,
            0.211450417,
            0.188953244,
            0.167259072,
            0.146354051,
            0.126224156,
            0.106855212,
            0.088232916,
            0.070342852,
            0.053170516,
            0.036701329,
            0.020920658,
            0.005813832,
            -0.008633841,
            -0.022437061,
            -0.035610519,
            -0.048168882,
            -0.060126782,
            -0.071498801,
            -0.082299462,
            -0.092543211,
            -0.102244412,
            -0.111417335,
            -0.120076144,
            -0.128234889,
            -0.135907497,
            -0.143107764,
            -0.149849348,
            -0.156145757,
            -0.162010348,
            -0.167456317,
            -0.172496695,
            -0.177144339,
            -0.181411931,
            -0.185311972,
            -0.188856774,
            -0.192058462,
            -0.194928965,
            -0.197480018,
            -0.199723152,
            -0.201669698,
            -0.203330783,
            -0.204717324,
            -0.205840032,
            -0.206709407,
            -0.207335737,
            -0.207729101,
            -0.207899362,
            -0.207856170,
            -0.207608964,
            -0.207166968,
            -0.206539194,
            -0.205734439,
            -0.204761292,
            -0.203628127,
            -0.202343110,
            -0.200914199,
            -0.199349141,
            -0.197655479,
            -0.195840553,
            -0.193911496,
            -0.191875245,
            -0.189738533,
            -0.187507899,
            -0.185189687,
            -0.182790049,
            -0.180314944,
            -0.177770148,
            -0.175161248,
            -0.172493651,
            -0.169772581,
            -0.167003090,
            -0.164190051,
            -0.161338168,
            -0.158451977,
            -0.155535845,
            -0.152593981,
            -0.149630431,
            -0.146649084,
            -0.143653680,
            -0.140647802,
            -0.137634891,
            -0.134618240,
            -0.131601002,
            -0.128586193,
            -0.125576692,
            -0.122575248,
            -0.119584478,
            -0.116606876,
            -0.113644811,
            -0.110700536,
            -0.107776183,
            -0.104873772,
            -0.101995214,
            -0.099142310,
            -0.096316756,
            -0.093520150,
            -0.090753986,
            -0.088019665,
            -0.085318495,
            -0.082651692,
            -0.080020386,
            -0.077425621,
            -0.074868360,
            -0.072349484,
            -0.069869800,
            -0.067430040,
            -0.065030863,
            -0.062672859,
            -0.060356553,
            -0.058082404,
            -0.055850809,
            -0.053662107,
            -0.051516576,
            -0.049414444,
            -0.047355882,
            -0.045341011,
            -0.043369906,
            -0.041442591,
            -0.039559050,
            -0.037719222,
            -0.035923004,
            -0.034170258,
            -0.032460806,
            -0.030794436,
            -0.029170902,
            -0.027589929,
            -0.026051208,
            -0.024554404,
            -0.023099155,
            -0.021685075,
            -0.020311751,
            -0.018978750,
            -0.017685619,
            -0.016431883,
            -0.015217050,
            -0.014040612,
            -0.012902045,
            -0.011800808,
            -0.010736351,
            -0.009708109,
            -0.008715507,
            -0.007757958,
            -0.006834870,
            -0.005945639,
            -0.005089657,
            -0.004266308,
            -0.003474972,
            -0.002715022,
            -0.001985832,
            -0.001286769,
            -0.000617199,
            0.000023511,
            0.000635998,
            0.001220897,
            0.001778842,
            0.002310468,
            0.002816406,
            0.003297286,
            0.003753734,
            0.004186372,
            0.004595822,
            0.004982697,
            0.005347608,
            0.005691160,
            0.006013953,
            0.006316580,
            0.006599631,
            0.006863685,
            0.007109317,
            0.007337095,
            0.007547579,
            0.007741322,
            0.007918869,
            0.008080755,
            0.008227512,
            0.008359658,
            0.008477706,
            0.008582160,
            0.008673515,
            0.008752256,
            0.008818861,
            0.008873798,
            0.008917526,
            0.008950495,
            0.008973146,
            0.008985911,
            0.008989211,
            0.008983462,
            0.008969065,
            0.008946417,
            0.008915903,
            0.008877900,
            0.008832774,
            0.008780884,
            0.008722579,
            0.008658199,
            0.008588075,
            0.008512530,
            0.008431877,
            0.008346420,
            0.008256457,
            0.008162273,
            0.008064149,
            0.007962355,
            0.007857153,
            0.007748798,
            0.007637536,
            0.007523604,
            0.007407234,
            0.007288647,
            0.007168060,
            0.007045679,
            0.006921704,
            0.006796329,
            0.006669739,
            0.006542114,
            0.006413625,
            0.006284437,
            0.006154710,
            0.006024595,
            0.005894239,
            0.005763781,
            0.005633355,
            0.005503089,
            0.005373105,
            0.005243519,
            0.005114442,
            0.004985979};

    for(int i = 0; i < RESPONSE_LENGTH; i++) {
        float filterResponse = hpfFilter.filter(1.0);
        ASSERT_NEAR(filterResponse, response[i], ACCEPTABLE_ERROR);
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
