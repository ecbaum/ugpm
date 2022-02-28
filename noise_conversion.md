# Noise conversion
The covariance of the noise is a required parameter when performing pre-integration.

Format of noise UGPM expects is variance in the respective unit of the sensor. That is variance for m/s^2 for accelerometer and variance for degrees/s for gyroscope. 

The noise of IMU data from the OXTS is provided in random walk [[1]](https://www.oxts.com/wp-content/uploads/2021/07/RT3000-v3-datasheet-210702.pdf) and more specifically 

    RW_acc:   0.005 m/s/sqrt(hr)
    RW_gyr:   0.2 degrees/sqrt(hr)


Sources in [[2]](https://www.vectornav.com/resources/inertial-navigation-primer/specifications--and--error-budgets/specs-imuspecs) explains the different versions of noise representation; standard deviation, noise density and random walk, and provides resources for how to convert between them [[3]](https://www.vectornav.com/resources/inertial-navigation-primer/examples/noise).

They state that the standard deviation, `STD`, can be found by multipling the random walk noise, `RW`, with the square root of time

    STD = RW*sqrt(t)

hence the variance is 

    VAR = STD^2 = RW^2*t

The time here is intepreted as time between samples of the IMU. OXTS outputs IMU data at 200 Hz which results in the following variances for the accelerometer and gyroscope respectively

    VAR_acc = 0.005^2/200 = 1.25e-7
    VAR_gyr = 0.2^2/200 = 2e-4