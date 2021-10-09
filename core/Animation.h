#ifndef ANIMATION_H
#define ANIMATION_H

#include <vector>

// Behavior at keyframe
enum TransitionType
{
    LINEAR_TRANSITION,
    EASE_TRANSITION,
    ABRUPT_TRANSITION,
    ABRUPT_IN_TRANSITION,
    ABRUPT_OUT_TRANSITION,
    NUM_TRANSITION_TYPES
};

template <class StateType>
struct KeyFrame
{
    StateType state;
    double duration;
    TransitionType transition;
    KeyFrame(StateType state, double duration, TransitionType transition) : state(state), duration(duration), transition(transition) {}

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template <class StateType>
class Animation : public std::vector<KeyFrame<StateType>>
{
public:
    // Returns the indices to the keyframes surrounding seconds_elapsed in the
    // current animations (a,b) and a fraction f ∈ [0,1] telling how much to
    // linearly interpolate between keyframes a and b.
    // Input:
    //   seconds_elapsed  seconds elapsed since the start of the animation
    // Output:
    //   a  index, into "this" Animation, such that KeyFrame *this[a] is the
    //      KeyFrame to be blending FROM at time seconds_elapsed, set to
    //      -1 if animation is empty
    //   b  index, into "this" Animation, such that KeyFrame *this[b] is the
    //      KeyFrame to be blending TO at time seconds_elapsed, set to
    //      -1 if animation is empty
    //   f  lerp fraction between 0 and 1, 0.0 means completely use KeyFrame
    //      *this[a], 1.0 means completely use KeyFrame *this[b], set to
    //      -1 if animation empty
    // Return:
    //   bool  true only if seconds_elapsed ∈ [0,∑*this[i].duration] and
    //     animation not empty
    //
    // This implementation is O(n) where n is the number of KeyFrames
    // This can certainly be optimized if the caller promises call sequentialy
    // in order. Then the implementation could be amortized O(1).
    bool get_frame(
        const double seconds_elapsed,
        size_t &a,
        size_t &b,
        double &f)
    {
        if (this->size() == 0)
        {
            a = -1;
            b = -1;
            f = -1;
            return false;
        }
        // Negative seconds elapsed --> use first keyframe for a and b
        if (seconds_elapsed < 0.0)
        {
            a = 0;
            b = 0;
            f = 0.0;
            return false;
        }
        // loop over key frames
        double seconds_so_far = 0.0;
        for (a = 0; a < this->size(); a++)
        {
            seconds_so_far += (*this)[a].duration;
            // bth keyframe happens after seconds_elapsed
            if (seconds_elapsed <= seconds_so_far)
            {
                break;
            }
        }
        // more than last frames duration
        if (a >= this->size())
        {
            a = this->size() - 1;
            b = this->size() - 1;
            f = 1.0;
            return false;
        }
        else if (a == this->size() - 1)
        {
            b = a;
        }
        else
        {
            b = a + 1;
        }
        f = 1.0 - (seconds_so_far - seconds_elapsed) / (*this)[a].duration;
        return true;
    }

public:
    double getDurationBefore(const int before)
    {
        assert(before <= (int)this->size());
        assert(before >= 0);
        double totDuration = 0.0;
        for (int i = 0; i < before; i++)
        {
            totDuration += (*this)[i].duration;
        }
        return totDuration;
    }
    double getTotalDuration()
    {
        return getDurationBefore(this->size());
    }

    // STATIC UTILITY METHODS
    // USEFULL FILTERS
public:
    // Filter function between 0 and 1 based on transition types at two
    // keyframes
    // Inputs:
    //   T0  transition type at first keyframe (when f=0)
    //   T1  transition type at second keyframe (when f=1)
    //   f  current function value to be filtered
    // Outputs:
    //   double  filtered function value
    static double filter(TransitionType T0, TransitionType T1, double f)
    {
        // filtered value according to A
        return (1 - f) * filter(T0, f) + f * filter(T1, f);
    }

    // Filter function based on given transition type
    static double filter(TransitionType T, double f)
    {
        switch (T)
        {
        case EASE_TRANSITION:
            return ease(f);
        case ABRUPT_TRANSITION:
            return abrupt(f);
        case ABRUPT_IN_TRANSITION:
            return abrupt_in(f);
        case ABRUPT_OUT_TRANSITION:
            return abrupt_out(f);
        case LINEAR_TRANSITION:
        default:
            return linear(f);
        }
    }

    static double linear(double f)
    {
        return f;
    }

    // Ease-in ease-out
    static double ease(double f)
    {
        return -2.0 * f * f * f + 3.0 * f * f;
    }

    // Abrupt-in, abrupt-out
    static double abrupt(double f)
    {
        return 2 * f * f * f - 3.0 * f * f + 2 * f;
    }

    static double abrupt_in(double f)
    {
        return pow(f, double(1.0 / 3.0));
    }

    static double abrupt_out(double f)
    {
        return f * f * f;
    }
};

#endif
