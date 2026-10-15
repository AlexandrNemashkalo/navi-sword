using Godot;

/// <summary>
/// The "1€ filter" (Casiez, Roussel, Vogel 2012) for three channels: a low-pass whose
/// cutoff rises with speed. Holding still it removes jitter; moving fast it adds
/// almost no lag.
/// </summary>
public sealed class OneEuroFilter
{
    private readonly float _minCutoff, _beta, _derivativeCutoff;
    private Vector3 _value, _derivative;
    private bool _primed;

    public OneEuroFilter(float minCutoff, float beta, float derivativeCutoff)
    {
        _minCutoff = minCutoff;
        _beta = beta;
        _derivativeCutoff = derivativeCutoff;
    }

    public void Reset()
    {
        _primed = false;
        _value = _derivative = Vector3.Zero;
    }

    public Vector3 Filter(Vector3 input, float dt)
    {
        if (!_primed || dt <= 0)
        {
            _primed = true;
            _value = input;
            _derivative = Vector3.Zero;
            return input;
        }
        _derivative = _derivative.Lerp((input - _value) / dt, Alpha(_derivativeCutoff, dt));
        Vector3 result = Vector3.Zero;
        for (int axis = 0; axis < 3; axis++)
        {
            float cutoff = _minCutoff + _beta * Mathf.Abs(_derivative[axis]);
            result[axis] = Mathf.Lerp(_value[axis], input[axis], Alpha(cutoff, dt));
        }
        _value = result;
        return result;
    }

    private static float Alpha(float cutoff, float dt)
    {
        float tau = 1 / (Mathf.Tau * cutoff);
        return 1 / (1 + tau / dt);
    }
}
