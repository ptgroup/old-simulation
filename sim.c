/*****TODO*****
 * Compare simulated data to real data
 * Implement fluctuations
 * Implement serial communication
 * LONG TERM:
 ** Implement beam
 ** Implement all the other functions (anneals, beam trips, etc.)
 **************/

/********POLARIZED TARGET SIMULATION********
 * File: sim.c
 * Version: 2.0
 * To compile: run 'make' in a terminal
 *
 * This program is intended to simulate the behavior
 * of the polarized target experiment, as performed
 * at UVA and JLAB, allowing for a variety of parameters
 * and a serial control interface (controller box interaction).
 *
 * The behavior of the simulation is controlled using a 'run file',
 * which is provided to the program at runtime and contains commands
 * allowing for various simulation parameters to be changed (or
 * for situations to be simulated, such as a beam trip).
 * Explanations of these commands are given below.
 *
 * By default, the simulation runs alongside the controller
 * box, feeding out data in real time; to disable this
 * and create simulation output as quickly as possible (for
 * graph creation, testing, etc.), put the line
 * 'serial off'
 * at the top of the run file.
 ********************************************/

/*****INPUT FILE COMMANDS*****
 * serial (on/off) - Turns the serial communications on or off
 *
 * init - Starts the initializer block
 **** rand (on/off) - Turns thermal fluctuations on/off
 **** annl (time) (temp) - Simulates a previous anneal
 **** mfld (field strength) - Sets the magnetic field strength
 **** temp (temperature) - Sets the temperature
 * done - Ends the initializer block
 *
 * freq (number) - Sets the frequency to <number> GHz
 * time (time) - Runs until the time <time> seconds
 * time +(time) - Runs for <time> seconds past the current time
 * beam (on/off) - Turns beam on/off
 * trip (time) - Simulates a beam trip for <time> seconds (ha6lf is trip, ha6lf is decay)
 * annl (time) (temp) - Anneals the material
 *****************************/

/*****UNITS*****
 * Polarization: as a fraction (-1 to 1)
 * Dose: in Pe/cm^2 (10^15 electrons / cm^2)
 * Time: in seconds
 * Frequency: in GHz
 * Field: in T
 * Temperature: in K
 ***************/

/*****MODEL*****
 * Polarization is modelled as a function of time:
 * P = P_infinity - A*exp(-lambda*t)
 * P_infinity = steady state polarization (function of frequency)
 * A = some constant (determined by initial polarization)
 * lambda = a rate constant (function of frequency)
 ***************/

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "helper.h"
#include "script.h"
#include "serial.h"

bool serial_on = false; // Whether to enable the serial interface (off by default)
const int PORT = 9; // Serial COM port - 1 (eg COM8 == 7)

// Simulation variables
double sim_time = 0.0; // In seconds
double freq = 140.145; // In GHz
double field = 5.0; // Field, in T
double temp = 1.0; // Temperature, in K

// Simulation control
const double DELTA_T = 1.0; // Simulated time step in seconds (NOT actual time step)
const double DELAY = 1.0; // Actual time step in seconds, when serial is on (NOT simulation time step)

// Simulation functions
void sim_init(); // Runs initialization for simulation
void run_until(double until); // Runs until a certain time
void update_sim(); // Advances the simulation by a time step of DELTA_T

// Dose variables (all dose values in 10e15 e- / cm^2)
const double MAX_DOSE_RATE = 0.0002; // Calculated from events3.csv
const double ANNEAL_DECAY_FACTOR = 0.9; // Decay is faster with more anneals
const double CDOSE_THRESHOLD[3] = {0.0, 0.3, 1.2}; // At what dose to change to the next critical dose
double critical_dose[3] = {1.0, 4.1, 30.}; // Formula for dose decay: P_0 * exp(-dose / crit_dose)
double dose_rate = 0.0; // The current rate of dose deposit (related to the beam current)
double last_anneal_dose = 0.0; // Dose at the last anneal
double dose = 0.0; // The current dose
int n_anneals = 0; // Number of anneals so far

// Polarization variables
double pol = 0.0; // The current polarization
double a_param = 1.0; // The A parameter from the model
double pol_rate; // The polarization rate, as obtained from the box

// Polarization functions
double optimal_freq_pos(); // Optimal frequency for polarizing positively
double optimal_freq_neg(); // Optimal frequency for polarizing negatively
double get_steady_state(); // Calculates P_infinity from the current frequency
double get_lambda(); // Calculates the parameter "lambda" from the current frequency
void update_a_param(); // Updates the A parameter (to be run every time the frequency is changed)
void update_pol(); // Updates the polarization (to be run after every time step)

// Frequency functions
void set_freq(double frequency); // Sets the frequency (also does other necessary calculations/adjustments)

// File I/O
FILE *output; // Data output
const size_t BUF_LEN = 200; // Length of buffer to read commands into
void output_data(); // Output data to file

// Box data
int direction; // The current motor direction

// Serial communications
void serial_init(); // Initializes serial comm.
void process_command(); // Receives and processes a command from the serial interface
// Various data commands
void rx_string(); // Allows for printing of arbitrary data from the box (receives null-terminated string)
void rx_freq();
void tx_confirmation();
void tx_event_num();
void rx_pol_rate();
void rx_direction();
void tx_pol();

int main(int argc, char **argv) {
    // Seed random number generator
    srand(time(NULL));

    bool allocated = false; // Whether we need to free the input_filename buffer
    char *input_filename;
    if (argc < 2) {
        // Prompt for an input filename
        printf("Script filename: ");
        input_filename = malloc(BUF_LEN*sizeof(char));
        allocated = true;
        fgets(input_filename, BUF_LEN, stdin);
        strip_newline(input_filename);
    } else if (argc == 2) {
        input_filename = argv[1];
    } else {
        puts("Too many arguments");
        return 1;
    }
    
    int failed = script_fopen(input_filename);
    if (failed) {
        printf("Could not open file: %s\n", input_filename);
        if (allocated) {
            free(input_filename);
        }
        return 1;
    }
    char *output_filename = input_filename;
    strip_extension(output_filename);
    strcat(output_filename, ".dat");
    output = fopen(output_filename, "w");
    // Free buffer if we used it
    if (allocated) {
        free(input_filename);
    }
    
    // Check for serial on/off line
    int read = script_readline();
    if (script_cmdequ("serial")) {
        if (!strcmp(script_getarg(0), "on")) {
            puts("Serial on");
            serial_on = true;
        } else if (!strcmp(script_getarg(0), "off")) {
            puts("Serial off");
            serial_on = false;
        } else {
            puts("Invalid serial instruction, continuing with serial off");
            serial_on = false;
        }
        // Get next command ready
        read = script_readline();
    }

    sim_init();
    puts("Initialized simulation");
    
    // Command loop
    do {
        // Command loop
        if (script_cmdequ("freq")) {
            double tmp;
            sscanf(script_getarg(0), "%6lf", &tmp);
            set_freq(tmp);
            printf("Set frequency: %6lf\n", freq);
        } else if (script_cmdequ("time")) {
            double until;
            sscanf(script_getarg(0), "%6lf", &until);
            printf("Running until time: %6lf\n", until);
            run_until(until);
        }
    } while ((read = script_readline()));
    
    // Close files and exit
    fclose(output);
    script_fclose();
    puts("Simulation finished successfully (press enter to exit)");
    getchar();
    
    return 0;
}

void sim_init() {
    // Make sure the necessary calculations are done at least once
    set_freq(freq);
    update_pol();
    // Initialize serial if necessary
    if (serial_on) {
        serial_init();
    }
}

void run_until(double until) {
    time_t old_time, curr_time; // For keeping track of the delay between updates
    time(&old_time);
    
    while (sim_time <= until) {
        time(&curr_time); // The current time (don't update until this is at least DELAY seconds after old_time)
        
        if (serial_on) {
            // Process any input commands
            process_command();
            // Wait until DELAY seconds before updating
            if (difftime(curr_time, old_time) >= DELAY) {
                update_sim();
                printf("Simulation time: %6lf\n", sim_time);
                // Reset "timer"
                old_time = curr_time;
            }
        } else {
            // Output old data first
            output_data();
            update_sim();
        }
    }
}

void update_sim() {
    double old_pol = pol;
    sim_time += DELTA_T;
    update_pol();
    
    // Update pol_rate if there is no serial to calculate it for us
    if (!serial_on) {
        pol_rate = (pol - old_pol) / DELTA_T;
    }
}

double optimal_freq_pos() {
    //"The positive polarization frequencies are more linear as they drift lower, from about 140.20 to near 140.13 GHZ in SANE."
    //From Polarized Sources, Targets and Polarimetry...Proceedings of the 13th Inernational Workshop. Pg. 151
    
    //Update 10/14/2015:
    //here is a curve for optimal POS freq based on SANE data 
    double A_pos = 140.1; //This is the "steady state" frequency
    double C_pos = 0.045; //This is the range; add this to A to get the initial frequency
    double k_pos = 0.38;  //This determines the decay rate
    
    return (A_pos + C_pos*exp(-k_pos*dose))*field/5.0;
}

double optimal_freq_neg() {
    //"In the case of DNP for negative polarization...a fast increase in the optimum microwave frequency which quickly slows, 
    //creating an exponential curve which...goes from 140.4 to around 150.53 GHz at the end of the anneal cycle (close to 4 Pe/cm^2)"
    //--same source as "optimal_freq_pos()"
    
    //Update 10/14/2015:
    //here is a curve for optimal NEG freq based on SANE data 
    double A_neg = 140.535; //This is the "steady state" frequency
    double C_neg = 0.065; //The range, subtract this from A to get the initial frequency
    double k_neg = 3.8; //This determines growth rate
    
    return (A_neg - C_neg*exp(-k_neg*dose))*field/5.0;
}

double get_steady_state() {
    // This is not based strictly on the data; a better model will be provided once better data is obtained
    double pos_diff = freq - optimal_freq_pos();
    double neg_diff = freq - optimal_freq_neg();
    // Modelled as pair of Gaussians with standard deviation 0.1 GHz
    return exp(-pos_diff*pos_diff/0.02) - exp(-neg_diff*neg_diff/0.02);
}

double get_lambda() {
    // This is not based strictly on the data; a better model will be provided once better data is obtained
    // Modelled as a Gaussian with mean as the average of optimal frequencies and standard deviation 0.15
    double m = 0.5*(optimal_freq_pos() + optimal_freq_neg());
    double dev = freq - m;
    return 0.005*exp(-dev*dev/0.045);
}

void update_a_param() {
    a_param = exp(get_lambda() * sim_time) * (get_steady_state() - pol);
}

void update_pol() {
    // TODO: Check that this model works
    pol = get_steady_state() - a_param * exp(-get_lambda() * sim_time);
}

void set_freq(double frequency) {
    freq = frequency;
    update_a_param();
}

void output_data() {
    if (serial_on) {
        puts("Writing to file");
        fprintf(output, "%6lf %6lf %6lf %6lf %6lf %6lf %6d\n", sim_time, freq, 100*pol, 100*get_steady_state(), get_lambda(), 100*pol_rate, direction);
    } else {
        // There will be no direction to output if we have serial off, so just put N/A in the column
        fprintf(output, "%6lf %6lf %6lf %6lf %6lf %6lf N/A   \n", sim_time, freq, 100*pol, 100*get_steady_state(), get_lambda(), 100*pol_rate);
    }
}

void serial_init() {
    serial_start(PORT);
}

void process_command() {
    if (!serial_on) return;
    
    uint8_t control;
    // Loop so that all available commands are processed
    while ((control = serial_rx_byte(PORT))) {
        switch((int)control) {
        case 0x11:
            puts("Reading frequency");
            rx_freq();
            break;
        case 0x33:
            puts("Confirmation requested");
            tx_confirmation();
            break;
        case 0x77:
            puts("Writing event number");
            tx_event_num();
            break;
        case 0x88:
            puts("Reading motor direction");
            rx_direction();
	        // The direction is the last bit of data to be
	        // sent by the box, so we know we have a complete
	        // row of data to output at this point
            output_data();
            break;
        case 0xBB:
            puts("Reading polarization rate");
            rx_pol_rate();
            break;
        case 0xEE:
            rx_string();
            break;
        case 0xFF:
            puts("Writing polarization");
            tx_pol();
            break;
        default:
            printf("Received unknown control byte: %hhX\n", control);
        }
    }
}

void rx_string() {
    printf("Message: \"");
    
    uint8_t c;
    while ((c = serial_rx_byte_wait(PORT)) != 0x0) {
        putchar(c);
    }
    
    printf("\"\n");
}

void rx_freq() {
    int32_t freq_int = serial_rx_int32(PORT);
    freq = (double)freq_int / 1000;
}

void tx_confirmation() {
    serial_tx_byte(PORT, 0xBE);
    serial_tx_byte(PORT, 0xEF);
}

void tx_event_num() {
    time_t event_time;
    time(&event_time);
    uint32_t event_num = (uint32_t)event_time;
    
    serial_tx_int32(PORT, event_num);
}

void rx_pol_rate() {
    pol_rate = (double)serial_rx_float(PORT);
}

void rx_direction() {
    direction = serial_rx_int32(PORT);
}

void tx_pol() {
    serial_tx_float(PORT, (float)pol);
}
