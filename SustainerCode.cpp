#include "daisy_seed.h"
#include "daisysp.h"
#include <math.h>

using namespace daisy;
using namespace daisysp;

struct FirstOrderFilter {
    float b0, b1, a1;
    float prev_x, prev_y;

    void Init(float g, float zero, float pole) {
        b0 = g;
        b1 = -g * zero;
        a1 = pole;
        prev_x = prev_y = 0.0f;
    }

    float Process(float x) {
        float y = b0 * x + b1 * prev_x + a1 * prev_y;
        prev_x = x;
        prev_y = y;
        return y;
    }
};

struct EnvelopeFollower {
    float attack, release, env;
    void Init(float fs) {
        attack = 0.99f; 
        release = 0.999f;
        env = 0.0f;
    }
    float Process(float in) {
        float v = fabsf(in);
        if (v > env) env = attack * env + (1.0f - attack) * v; 
        else         env = release * env + (1.0f - release) * v;
        return env;
    }
};

DaisySeed        hw;
EnvelopeFollower env_fol;
FirstOrderFilter lpFilter, hpFilter; 
GPIO             sw1_pin, sw2_pin;               

// Using Interleaving buffers to match your working snippet's structure
void AudioCallback(AudioHandle::InterleavingInputBuffer  in,
                   AudioHandle::InterleavingOutputBuffer out,
                   size_t                                size)
{
    for (size_t i = 0; i < size; i += 2)
    {
        float input_signal = in[i];
        
        float envelope = env_fol.Process(input_signal);
        float gain     = 1.0f - envelope;

        bool s1 = sw1_pin.Read();
        bool s2 = sw2_pin.Read();

        float drive_sig;
        if (!s1 && s2) {
            drive_sig = gain;
        } else if (s1 && !s2) {
            drive_sig = fabsf(gain) * 2.0f - 1.0f;
        } else {
            drive_sig = (gain + (fabsf(gain) * 2.0f - 1.0f)) * 0.5f;
        }
        
        float saturated_gain = tanhf(drive_sig);

        float filtered = lpFilter.Process(input_signal);
        filtered       = hpFilter.Process(filtered);
        
        float final_out = filtered * saturated_gain;

        out[i]     = final_out;
        out[i + 1] = final_out;
    }
}

int main(void)
{
    hw.Init();
    float fs = hw.AudioSampleRate();

    // Initialize DSP objects
    env_fol.Init(fs);
    lpFilter.Init(0.366f, -1.0f, 0.268f);
    hpFilter.Init(0.9974f, 1.0f, 0.9948f);

    // Initialize Hardware Peripherals
    sw1_pin.Init(seed::D24, GPIO::Mode::INPUT, GPIO::Pull::PULLUP);
    sw2_pin.Init(seed::D25, GPIO::Mode::INPUT, GPIO::Pull::PULLUP);

    // Audio Configuration
    hw.SetAudioBlockSize(4);
    // Note: StartAudio defaults to interleaved if the callback signature matches
    hw.StartAudio(AudioCallback);

    bool led_state = true;
    while(1) {
        hw.SetLed(led_state);
        hw.DelayMs(200);
        led_state = !led_state;
    }
}