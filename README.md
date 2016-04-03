# still

*wait until the LSM9DS0 accelerometer detects movement, then
optionally run a command*

```c++
/*
 * still [options] [[--] command [args...]]
 *
 * Poll LSM9DS0 accelerometer and (optionally) run a command when it detects movement.
 * If command is not specified, exit instead.
 *
 * Specifying a command:
 * --: separate still's options from the command to run
 * command: the command to run
 * args...: additional arguments to the command
 *
 * Configuring the trigger:
 * --buffer n: set the size of the accelerometer sample buffer to n
 * --discard ms: discard all sensor readings for the first ms milliseconds
 * --threshold t: trigger threshold for deviation from calibrated mean,
 * 		as a fraction of the calibrated mean magnitude
 *
 * Using the watchdog timer
 * --watchdog: open /dev/watchdog and write to it for every sample
 * --timeout t: set the trigger timeout for /dev/watchdog, implies --watchdog
 */
```

Uses driver code from [SparkFun's 9DOF driver][9dof-driver] to monitor
an LSM9DS0 from a [SparkFun 9DOF block][9dof-block]'s accelerometer,
waiting for movement.  If a command is supplied, runs it.  Otherwise returns.

Useful as a physical security defense with the `--watchdog` option, which
causes the edison to hard reboot and thus lose any sensitive information
in its RAM.

Requires a recent version of [Intel's MRAA library][mraa] for the SparkFun driver.
Requires [Boost Program Options][boost_po] (`apt-get libboost_program_options`)
to process command-line arguments.  Run it as the `root` user on your Edison
or it will flagrantly fail to work.

[9dof-driver]: https://github.com/sparkfun/SparkFun_9DOF_Block_for_Edison_CPP_Library
[9dof-block]: https://www.sparkfun.com/products/13033
[mraa]: https://github.com/intel-iot-devkit/mraa
[boost_po]: http://www.boost.org/doc/libs/release/libs/program_options/
