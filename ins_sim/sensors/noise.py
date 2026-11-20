"""Random processes shared by the sensor error models.

`gauss_markov` draws the first-order Gauss-Markov process that ins_sim uses
for every slowly wandering sensor error: the in-run bias of the gyros and
accelerometers, and the weather drift of the barometric altimeter.
"""

import numpy as np
from scipy.signal import lfilter


def gauss_markov(rng: np.random.Generator, n_samples: int, tau_i: float,
                 sigma: float, tau_c: float, n_axes: int = 3) -> np.ndarray:
    """Draws a stationary first-order Gauss-Markov process on each axis.

    The discrete recursion is

        x[k+1] = a·x[k] + √(1 − a²)·σ·u[k+1],   a = exp(−tau_i/tau_c),

    with u white and unit-variance. The process starts in its stationary
    distribution, x[0] ~ N(0, σ²), so its spread is σ at every sample and
    its autocorrelation is exp(−|Δt|/tau_c).

    The draws come from `rng` in a fixed order, x[0] then u, and happen
    whatever σ is, so a zero σ consumes the same random numbers as any
    other value.

    Args:
        rng: Seeded NumPy random generator.
        n_samples: Number of samples M.
        tau_i: Sample interval [s].
        sigma: Steady-state standard deviation σ, in the process's units.
        tau_c: Correlation time [s], above 0.
        n_axes: Number of independent axes. Defaults to 3.

    Returns:
        numpy.ndarray: The process, shape (n_samples, n_axes).

    Examples:
        >>> import numpy as np
        >>> from ins_sim.sensors.noise import gauss_markov
        >>> x = gauss_markov(np.random.default_rng(0), 5, tau_i=1.0,
        ...                  sigma=2.0, tau_c=60.0, n_axes=1)
        >>> x.shape
        (5, 1)
    """
    # --8<-- [start:gauss_markov]
    # The IIR filter produces the zero-start response to u; the decaying
    # initial state a^k·x[0] is added to it.
    a = np.exp(-tau_i / tau_c)
    s = np.sqrt(1.0 - a * a) * sigma
    x0 = rng.normal(0.0, sigma, size=n_axes)
    u = rng.normal(size=(n_samples, n_axes)); u[0] = 0.0
    return (lfilter([1.0], [1.0, -a], s * u, axis=0)
            + a ** np.arange(n_samples)[:, None] * x0)
    # --8<-- [end:gauss_markov]
