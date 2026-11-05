"""
Provides the core mathematical implementation of the Unscented Kalman Filter, 
using the scaled unscented transform (Van der Merwe / Wan formulation). It handles 
sigma point generation, non-linear state prediction, and measurement updates
To be used by ukf_predictor.py
"""

# Import necessary libraries
import numpy as np


class UnscentedKalmanFilter:
    def __init__(self, dim_x, dim_z, fx, hx, alpha=1e-3, beta=2.0, kappa=0.0):
        """
        Args:
            dim_x (int): The dimension of the state vector (n).
            dim_z (int): The dimension of the measurement vector (m).
            fx (callable): The process model function, signature fx(x, dt) -> x_next.
            hx (callable): The measurement model function, signature hx(x) -> z.
            alpha (float): Spread of the sigma points around the mean.
            beta (float): Incorporates prior knowledge of the distribution (2.0 for Gaussian).
            kappa (float): Secondary scaling parameter.
        """
        self.n = dim_x
        self.m = dim_z
        self.fx = fx
        self.hx = hx

        self.alpha = alpha
        self.beta = beta
        self.kappa = kappa

        # Calculate the scaling factor lambda
        self.lam = alpha ** 2 * (self.n + kappa) - self.n

        self.x = np.zeros(self.n)          # state estimate
        self.P = np.eye(self.n)            # state covariance
        self.Q = np.eye(self.n)            # process noise covariance
        self.R = np.eye(self.m)            # measurement noise covariance
        self.sigmas_f = None               # Propagated sigma points (cached for the update step)

        # Pre-compute the weights for the unscented transform
        self._compute_weights()



    def _compute_weights(self):
        """
        Calculates the weights used to recombine the sigma points into the new 
        mean (Wm) and covariance (Wc) estimates.
        """
        n, lam = self.n, self.lam
        c = 1.0 / (2.0 * (n + lam))

        self.Wm = np.full(2 * n + 1, c)
        self.Wc = np.full(2 * n + 1, c)

        # The weight for the 0th (first mean) sigma point is calculated differently
        self.Wm[0] = lam / (n + lam)
        self.Wc[0] = lam / (n + lam) + (1.0 - self.alpha ** 2 + self.beta)



    def _sigma_points(self, x, P):
        """
        Generates 2n+1 sigma points distributed around the current state estimate 
        based on the covariance matrix P.
        Args:
            x (np.array): Current state mean.
            P (np.array): Current state covariance.
        Returns:
            np.array: A matrix of sigma points.
        """

        n, lam = self.n, self.lam
        sigmas = np.zeros((2 * n + 1, n))
        sigmas[0] = x
        U = np.linalg.cholesky((n + lam) * P)
        for i in range(n):
            sigmas[i + 1] = x + U[:, i]
            sigmas[n + i + 1] = x - U[:, i]
        return sigmas



    def predict(self, dt, u=None):
        """
        The prediction step of the UKF.
        Generates sigma points, propagates them through the non-linear process 
        model (fx), and calculates the predicted prior state mean and covariance.
        Args:
            dt (float): Time step duration.
            u (np.array): Control input vector.
        """
        sigmas = self._sigma_points(self.x, self.P)

        # Propagate each sigma point through the process model
        if u is None:
            sigmas_f = np.array([self.fx(s, dt) for s in sigmas])
        else:
            sigmas_f = np.array([self.fx(s, dt, u) for s in sigmas])

        # Recombine to find the predicted mean and covariance
        x_pred = self.Wm @ sigmas_f
        P_pred = self.Q.copy()
        for i in range(2 * self.n + 1):
            y = sigmas_f[i] - x_pred
            P_pred += self.Wc[i] * np.outer(y, y)
        self.sigmas_f = sigmas_f
        self.x = x_pred
        self.P = P_pred

        # Enforce covariance matrix symmetry to prevent the UKF from crashing 
        # due to accumulated floating-point rounding errors during repeated matrix multiplications.
        self.P = (self.P + self.P.T) / 2.0
        self.P += np.eye(self.n) * 1e-5



    def update(self, z):
        """
        The measurement update step of the UKF.
        Propagates the predicted sigma points through the non-linear measurement 
        model (hx), calculates the Kalman gain, and updates the state estimate 
        and covariance based on the actual measurement z.
        Args:
            z (np.array): The actual measurement received from the sensor.
        """
        if self.sigmas_f is None:
            raise RuntimeError("predict() must be called before update()")

        # Propagate the already-predicted sigma points through the measurement model
        sigmas_h = np.array([self.hx(s) for s in self.sigmas_f])

        # Calculate the predicted measurement mean
        z_pred = self.Wm @ sigmas_h


        # Calculate measurement covariance (Pzz) and state-measurement cross-covariance (Pxz)
        Pzz = self.R.copy()
        Pxz = np.zeros((self.n, self.m))
        for i in range(2 * self.n + 1):
            dz = sigmas_h[i] - z_pred
            dx = self.sigmas_f[i] - self.x
            Pzz += self.Wc[i] * np.outer(dz, dz)
            Pxz += self.Wc[i] * np.outer(dx, dz)

        # Extract and stash the innovation vector (self.y) and its 
        # covariance matrix (self.S) as class attributes. 
        self.y = np.asarray(z) - z_pred
        self.S = Pzz.copy()
        
        # Calculate Kalman Gain (K)
        K = Pxz @ np.linalg.inv(Pzz)

        # update the state estimate and covariance
        self.x = self.x + K @ self.y
        self.P = self.P - K @ Pzz @ K.T

        # Enforce covariance matrix symmetry post-update and added a small positive 
        # term to the diagonal to ensure the matrix remains positive-definite, 
        # handling accumulated floating-point errors
        self.P = (self.P + self.P.T) / 2.0
        self.P += np.eye(self.n) * 1e-5

        # Implemente eigenvalue clipping to strictly enforce the positive-definiteness 
        # of the covariance matrix. Any negative eigenvalues caused by numerical instability 
        # are floored to a tiny positive number (1e-8) 
        vals, vecs = np.linalg.eigh(self.P)
        vals = np.maximum(vals, 1e-8) 
        self.P = vecs @ np.diag(vals) @ vecs.T

        # Nullify the cached `sigmas_f` after a successful update
        # to prevent the filter from updating twice in a row if a second measurement 
        #arrives before the next predict() cycle
        self.sigmas_f = None
