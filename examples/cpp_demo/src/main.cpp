// cpp_demo xcplib C++ example

#include <atomic>   // for std::atomic
#include <csignal>  // for signal(), SIGINT, SIGTERM
#include <cstdint>  // for uintxx_t
#include <cstring>  // for memset
#include <iostream> // for std::cout
#include <thread>   // for std::thread

#include "a2l.hpp"    // for xcplib A2l generation application programming interface
#include "xcplib.hpp" // for xcplib application programming interface

#include "lookup.hpp"  // for lookup_table::LookupTableT
#include "sig_gen.hpp" // for signal_generator::SignalGenerator

// NIRA added headers
#include <stdlib.h>
#include <unistd.h>
#include <ctype.h>
#include <getopt.h>
#include <fstream>

//-----------------------------------------------------------------------------------------------------
// XCP parameters
#define OPTION_PROJECT_NAME "cpp_demo"  // A2L project name
#define OPTION_PROJECT_EPK "NIRA XCPLite" // EPK version string
#define OPTION_USE_TCP false            // TCP or UDP
#define OPTION_SERVER_PORT 5555         // Port
#define OPTION_SERVER_ADDR {0, 0, 0, 0} // Bind addr, 0.0.0.0 = ANY
#define OPTION_QUEUE_SIZE (1024 * 256)  // Size of the measurement queue in bytes
#define OPTION_LOG_LEVEL 4              // Log level, 0 = no log, 1 = error, 2 = warning, 3 = info, 4 = debug

//-----------------------------------------------------------------------------------------------------
// Demo calibration parameters

struct ParametersT {
    uint16_t counter_max; // Maximum value for the counter
    uint32_t delay_us;    // Sleep time in microseconds for the main loop
};

// Default values
const ParametersT kParameters = {.counter_max = 1000, .delay_us = 20000};

//-----------------------------------------------------------------------------------------------------
// Demo global measurement values

uint8_t temperature = 50; // In Celsius
double speed = 0.0f;      // Speed in km/h

// NIRA VARIABLES
static int8_t AxleHeightRear = 100;
static uint8_t SuspensionOperationMode = 2;
static int16_t LatAcc = 100;
static int16_t LonAcc = -100;
static uint16_t Velocity = 10000;
static uint32_t Odometer = 1000;
static uint64_t GpsTime = 1234567;
static double AmbientTemperature = 20.0;
static float EngineTorque = 1250.0;
static bool BrakingInProgress = true;

static uint32_t datalog[100];

//-----------------------------------------------------------------------------------------------------
// Demo signal generator class

constexpr double kPi = 3.14159265358979323846;
constexpr double k2Pi = (kPi * 2);

// Default parameter values for multiple instances
const signal_generator::SignalParametersT kSignalParameters1 = {
    .ampl = 12.5,
    .phase = 0.0,
    .offset = 0.0,
    .period = 0.4, // s
    .lookup =
        {
            .values = {0.0f, 0.5f, 1.0f, 0.50f, 0.0f, -0.5f, -1.0f, -0.5f, 0.0f, 0.0f, 0.0f},
#ifdef CANAPE_24
            .lookup_axis = {0.0f, 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f, 0.9f, 1.0f},
#endif
        },
    .delay_us = 1000,                                   // us
    .signal_type = signal_generator::SignalTypeT::SINE, // Type of the signal
};
const signal_generator::SignalParametersT kSignalParameters2 = {
    .ampl = 80.0,
    .phase = kPi / 2,
    .offset = 0.0,
    .period = 10.0, // s
    .lookup =
        {
            .values = {0.0f, 0.1f, 0.3f, 0.6f, 0.8f, 1.0f, 0.80f, 0.6f, 0.3f, 0.1f, 0.0f},
#ifdef CANAPE_24
            .lookup_axis = {0.0f, 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f, 0.9f, 1.0f},
#endif
        },
    .delay_us = 1000,                                   // us
    .signal_type = signal_generator::SignalTypeT::SINE, // Type of the signal
};

//-----------------------------------------------------------------------------------------------------

// Signal handler for clean shutdown
static volatile bool running = true;
static void sig_handler(int sig) { running = false; }

/* NIRA STUFF */
enum PROTOCOL {
    UNDEFINED,
    TCP,
    UDP
};

struct RunArguments {
    enum PROTOCOL protocol;
    uint16_t port;
    char xcp_file[1024];
};

static bool parse_args(int argc, char *argv[], struct RunArguments* parsed_args) {
    int opt;
    int port = -1;

    // Define long options
    static struct option long_options[] = {
        {"protocol", required_argument, 0, 'p'},
        {"port",     required_argument, 0, 'P'},
        {"help",     no_argument,       0, 'h'},
        {0, 0, 0, 0}
    };

    while ((opt = getopt_long(argc, argv, "p:P:f:h", long_options, NULL)) != -1) {
        switch (opt) {
            case 'p': {
                if (strcasecmp(optarg, "TCP") == 0)
                {
                    parsed_args->protocol = TCP;
                }
                else if (strcasecmp(optarg, "UDP") == 0) {
                    parsed_args->protocol = UDP;
                } else {
                    fprintf(stderr, "Error: protocol must be TCP or UDP\n");
                    return false;
                }
                break;
            }

            case 'P': {
                port = atoi(optarg);
                if (port <= 0 || port > 65535) {
                    fprintf(stderr, "Error: port must be a valid integer between 1 and 65535\n");
                    return false;
                }
                parsed_args->port = port;
                break;
            }

            case 'f': {
                strncpy(parsed_args->xcp_file, optarg, sizeof(parsed_args->xcp_file) - 1);
                break;
            }

            case 'h':
            default:
                printf("Usage: %s --protocol TCP|UDP --port <number> --file <path_to_xcp_data_file>\n", argv[0]);
                return false;
        }
    }

    // Validate required arguments
    if (parsed_args->protocol == UNDEFINED) {
        fprintf(stderr, "Error: protocol is required\n");
        return false;
    }

    if (port == -1) {
        fprintf(stderr, "Error: port is required\n");
        return false;
    }

    return true;
}

int main(int argc, char* argv[]) {
    struct RunArguments run_args;
    if (!parse_args(argc, argv, &run_args))
    {
        return 1;
    }

    const bool datalog_provided = run_args.xcp_file[0] != '\0';
    std::ifstream stream;
    if (datalog_provided)
    {
        // Initialize for reading
        stream.open(run_args.xcp_file);
        // Move to start
        stream.seekg(28+140*400);
    }

    std::cout << "\nXCP on Ethernet cpp_demo C++ xcplib demo\n" << std::endl;
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    // Set log level (1-error, 2-warning, 3-info, 4-show XCP commands)
    XcpSetLogLevel(OPTION_LOG_LEVEL);

    // Initialize the XCP singleton, activate XCP, must be called before starting the server
    // If XCP is not activated, the server will not start and all XCP instrumentation will be passive with minimal overhead
    XcpInit(OPTION_PROJECT_NAME, OPTION_PROJECT_EPK, true);

    // Initialize the XCP Server
    uint8_t addr[4] = OPTION_SERVER_ADDR;
    if (!XcpEthServerInit(addr, run_args.port, run_args.protocol == TCP, OPTION_QUEUE_SIZE)) {
        std::cerr << "Failed to initialize XCP server" << std::endl;
        return 1;
    }

    // Enable A2L generation
    // Set mode to write once to create stable A2L files, this also enables calibration segment persistence and freeze support
    if (!A2lInit(addr, run_args.port, run_args.protocol == TCP, A2L_MODE_WRITE_ONCE | A2L_MODE_FINALIZE_ON_CONNECT | A2L_MODE_AUTO_GROUPS)) {
        std::cerr << "Failed to initialize A2L generator" << std::endl;
        return 1;
    }

    // Create a calibration segment wrapper for the struct 'ParametersT' and use its default values in constant 'kParameters'
    // This calibration segment has a working page (RAM) and a reference page (FLASH), it creates a MEMORY_SEGMENT in the A2L file
    // It provides safe (thread safe against XCP modifications), lock-free and consistent access to the calibration parameters
    // It supports XCP/ECU independent page switching, checksum calculation and reinitialization (copy reference page to working page)
    auto calseg = xcplib::CreateCalSeg("kParameters", &kParameters);

    // Add the calibration segment description as a typedef instance to the A2L file
    A2lTypedefBegin(ParametersT, &kParameters, "A2L Typedef for ParametersT");
    A2lTypedefParameterComponent(counter_max, "Maximum counter value", "", 0, 2000);
    A2lTypedefParameterComponent(delay_us, "Mainloop delay time in us", "us", 0, 999999);
    A2lTypedefEnd();
    calseg.CreateA2lTypedefInstance("ParametersT", "Main parameters");

    // Local variables
    uint16_t counter = 0;
    uint64_t loop_time = 0;
    uint64_t loop_cycletime = 0;
    constexpr size_t kHistogramSize = 256;
    uint32_t loop_histogram[kHistogramSize];
    memset(loop_histogram, 0, sizeof(loop_histogram));
    double sum = 0, channel1 = 0, channel2 = 0;

    // Create a measurement event 'mainloop'
    DaqCreateEvent(mainloop);

    // Register the global measurement variables 'temperature' and 'speed'
    A2lSetAbsoluteAddrMode(mainloop);
    A2lCreateLinearConversion(temperature, "Temperature in °C from unsigned byte", "°C", 1.0, -50.0);
    A2lCreatePhysMeasurement(temperature, "Motor temperature in °C", "conv.temperature", -50.0, 200.0);
    A2lCreatePhysMeasurement(speed, "Speed in km/h", "km/h", 0, 250.0);

    // Register NIRA variables
    A2lCreatePhysMeasurement(AxleHeightRear, "Axle height on rear axle", A2lCreateLinearConversion(AxleHeightRear, "Conversion for axle height", "mm", 0.5, 0.0), 0.0, 0.0);
    A2lCreateMeasurement(SuspensionOperationMode, "Suspension ");
    A2lCreatePhysMeasurement(LatAcc, "Longitudinal acceleration", A2lCreateLinearConversion(LatAcc, "Conversion for lateral acceleration", "m/s2", 1.0/128.0, 0.0), 0.0, 0.0);
    A2lCreatePhysMeasurement(LonAcc, "Longitudinal acceleration", A2lCreateLinearConversion(LonAcc, "Conversion for longitudinal acceleration", "m/s2", 1.0/128.0, 0.0), 0.0, 0.0);
    A2lCreatePhysMeasurement(Velocity, "Vehicle velocity", A2lCreateLinearConversion(Velocity, "Conversion for velocity", "m/s", 1.0/128.0, 0.0), 0.0, 0.0);
    A2lCreatePhysMeasurement(Odometer, "Odometer", A2lCreateLinearConversion(Odometer, "Conversion for odometer", "m", 1000.0, 0.0), 0.0, 0.0);
    A2lCreatePhysMeasurement(GpsTime, "GNSS Time", A2lCreateLinearConversion(GpsTime, "Conversion GPSTime", "s", 0.5, 0.0), 0.0, 0.0);
    A2lCreatePhysMeasurement(AmbientTemperature, "Outside air temperature", "degC", 0.0, 0.0);
    A2lCreatePhysMeasurement(EngineTorque, "Engine torque", "Nm", 0.0, 0.0);
    A2lCreateMeasurement(BrakingInProgress, "Braking in progress flag");

    for (int i = 0; i < sizeof(datalog)/sizeof(datalog[0]); i++)
    {
        char buf[20];
        sprintf(buf, "datalog_%d", i);
        A2lCreateMeasurementInstance(buf, datalog[i], "Datalog array");
    }

    // Register the local measurement variables 'loop_counter', 'loop_time', 'loop_cycletime', 'loop_histogram' and 'sum'
    A2lSetStackAddrMode(mainloop);
    A2lCreateMeasurement(counter, "Mainloop loop counter");
    A2lCreateLinearConversion(clock_ticks, "Conversion from clock ticks to milliseconds", "ms", 1.0 / 1000.0, 0.0);
    A2lCreatePhysMeasurement(loop_cycletime, "Mainloop cycle time", "conv.clock_ticks", 0.0, 0.05);
    A2lCreateMeasurementArray(loop_histogram, "Mainloop cycle time histogram");
    A2lCreateMeasurement(sum, "Sum of SigGen1 and SigGen2 value");

    // Signal generator C++ class demo
    // See sig_gen.cpp for details how to measure instance members and stack variables in member functions
    // Create 2 signal generator instances of class SignalGenerator with individual parameters
    // Note that the signal generator threads register measurements in the A2L file as well
    // This is not in conflict because the main thread has already registered its measurements above
    // Otherwise use A2lLock() and A2lUnlock() to avoid race conditions when registering measurements, the A2L generator macros for are not thread safe by itself
    signal_generator::SignalGenerator signal_generator_1("SigGen1", &kSignalParameters1);
    signal_generator::SignalGenerator signal_generator_2("SigGen2", &kSignalParameters2);

    sleepUs(100000);
    A2lFinalize(); // @@@@ TEST: Manually finalize the A2L file to make it visible without XCP tool connect

    // Main loop
    std::cout << "Starting main loop..." << std::endl;
    while (running) {
        // Access the calibration parameters 'delay' and 'counter_max' safely
        // Use RAII guard for automatic lock/unlock the calibration parameter segment 'calseg'
        {
            auto parameters = calseg.lock();

            // Increment the local measurement variable 'loop_counter' using the calibration parameter 'counter_max' as a limit
            counter++;
            if (counter > parameters->counter_max)
                counter = 0;

        } // Guard automatically unlocks here

        // Measure and calculate the mainloop cycle time
        uint64_t last_loop_time = loop_time;
        loop_time = clockGetUs();
        loop_cycletime = loop_time - last_loop_time;
        loop_histogram[loop_cycletime >= (1000000 / 10) * (kHistogramSize - 1) ? (kHistogramSize - 1) : loop_cycletime / (1000000 / 10)]++;

        // Sum the values of signal generator 1+2 into the local variable sum
        channel1 = signal_generator_1.GetValue();
        channel2 = signal_generator_2.GetValue();
        sum = channel1 + channel2;

        // Update some more local and global variables
        if (counter == 0) {
            temperature += 1;
            if (temperature > 150)
                temperature = 0; // Reset temperature to -50 °C
        }
        speed += (250.0f - speed) * 0.0001;
        if (speed > 245.0f)
            speed = 0; // Reset speed to 0 km/h

        
        if (datalog_provided)
        {
            if (XcpIsDaqRunning())
            {
                // Update the data array until file is exhausted
                if (!stream.eof())
                {
                    stream.read(reinterpret_cast<char*>(&datalog[0]), 400);
                }
            }
            else if (stream.tellg() != (28+140*400))
            {
                // Reset when not logging
                stream.seekg(28+140*400);
            }
        }

        // Trigger the XCP measurement mainloop for temperature, speed, loop_counter and sum
        DaqTriggerEvent(mainloop);

        sleepUs(calseg.lock()->delay_us);
    } // while (running)

    // Cleanup
    XcpDisconnect();
    XcpEthServerShutdown();

    return 0;
}
