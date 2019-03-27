# https://martin-thoma.com/twiddle/
# Choose an initialization parameter vector
p = [0, 0, 0]
# Define potential changes
dp = [1, 1, 1]
# Calculate the error
best_err = A(p)

threshold = 0.001
def get_error(p):


while sum(dp) > threshold:
    for i in range(len(p)):
        p[i] += dp[i]
        err = A(p)

        if err < best_err:  # There was some improvement
            best_err = err
            dp[i] *= 1.1
        else:  # There was no improvement
            p[i] -= 2*dp[i]  # Go into the other direction
            err = A(p)

            if err < best_err:  # There was an improvement
                best_err = err
                dp[i] *= 1.05
            else:  # There was no improvement
                p[i] += dp[i]
                # As there was no improvement, the step size in either
                # direction, the step size might simply be too big.
                dp[i] *= 0.95