/********POLARIZED TARGET SIMULATION********
 * File: sim.c
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

 * init - Starts the initializer block
 **** rand (on/off) - Turns thermal fluctuations on/off
 **** annl (time) (temp) - Simulates a previous anneal
 **** mfld (field strength) - Sets the magnetic field strength
 **** temp (temperature) - Sets the temperature
 **** sdst (steady state) - Sets the steady state of the polarization
 * done - Ends the initializer block
 *
 * freq (number) - Sets the frequency to <number> GHz
 * time (time) - Runs until the time <time> seconds
 * time +(time) - Runs for <time> seconds past the current time
 * beam (on/off) - Turns beam on/off
 * trip (time) - Simulates a beam trip for <time> seconds (half is trip, half is decay)
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

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "script.h"
#include "serial.h"

bool serial_on = true; // Whether to enable the serial interface (on by default)
const int PORT = 9; // Serial COM port - 1 (eg COM8 == 7)

// Polarization
const double POS_NEG_DIFFERENTIATOR = 140.3; // Frequency that differentiates b/w pos. and neg. polarization
double max_steady_state = 0.95; // Maximum possible polarization at 1K
double max_pol_rate = 0.001314; // Maximum rate of polarization increase per second
double pol = 0.0; // The current polarization
double pol_rate = 0.0; // The rate of polarization increase
double steady_state = 0.95; // Steady state polarization
double old_steady_state = 0; // For coming back from a beam trip
double direction = 99;
double k_val; // This dictates the rate of polarization increase/decrease


// Dose (all dose values in 10e15 e- / cm^2)
const double MAX_DOSE_RATE = 0.0002; // Calculated from events3.csv
const double ANNEAL_DECAY_FACTOR = 0.9; // Decay is faster with more anneals
const double CDOSE_THRESHOLD[3] = {0.0, 0.3, 1.2}; // At what dose to change to the next critical dose
double critical_dose[3] = {1.0, 4.1, 30.}; // Formula for dose decay: P_0 * exp(-dose / crit_dose)
double dose_rate = 0.0; // The current rate of dose deposit (related to the beam current)
double last_anneal_dose = 0.0; // Dose at the last anneal
double dose = 0.0; // The current dose
int n_anneals = 0; // Number of anneals so far

// Simulation variables
double sim_time = 0.0; // In seconds
double freq = 140.145; // In GHz
double field = 5.0;
double temp = 1.0;
bool tripping = false; // Whether we're in a beam trip or not

// Simulation setup
const double DELTA_T = 1.0; // Time step between outputs (simulated time)
const int DELAY = 1; // Number of seconds between outputs (actual time)
const int N_ITER = 2000; // Number of iterations per time step
const int BASE_RANDOMNESS = 500; // Base value for randomness (sort of an arbitrary value)
bool randomness_on = true;
bool follow_freq = false; // Whether to follow the ideal frequency

// Polarization functions
double optimal_freq_pos();
double optimal_freq_neg();
double deviation_increasing(double freq_diff);
double deviation_decreasing(double freq_diff);
void update_steady_state(double delta_t);
void reset_steady_state(); // Resets the steady state based on temperature
double get_steady_state(double deviation); // Gets the steady state, adjusted for frequency
void update_pol(double delta_t); // Updates the polarization by a time delta_t

// Main simulation functions
void update(); // Calculates everything for one DELTA_T time step
void run_until(double new_time); // Runs simulation continuously until new_time
void beam_on(double rate); // Turns beam on
void beam_off(); // Turns beam off
void anneal(double annl_time, double temp); // Simulates anneal
void output_data(); // Outputs a line of data to the file

// Output and helper functions
FILE *output; // Data output
FILE *commands; // Input (scenario)
const size_t BUF_LEN = 200; // Length of buffer to read commands into
int rand_int(int min, int max); // Generates a random integer n with min <= n < max
void delay(int seconds); // Delays for a time in seconds

// Communication functions (with controller box)
void process_command(); // Processes a command if there is one available

void sendConfirmation(); // sends a confirmation code (confirms connection)
void readPolRate();
void readFreq(); // reads frequency from serial (should be MSB first)
void readDirection(); //reads direction from serial
void writeFreq(); // sends frequency out (same format as for reading)
void writePol(); // sends polarization out (two bytes, MSB first, sends 10000*Pn)
void writeEventNum(); // sends eventnum out (four bytes, MSB first)

int main(int argc, char **argv) {
    // Whether we already processed the init block or are in it
    bool did_init = false;
    bool in_init = false;
    
    // Seed the random number generator
    srand(time(NULL));
    
    char *input_filename;
    bool allocated = false;

    // If a run file was not provided as a command-line argument,
    // we should prompt for it
    if (argc < 2) {
        input_filename = malloc(BUF_LEN * sizeof(char));
        allocated = true;
        printf("Enter the filename of the script: ");
        fgets(input_filename, BUF_LEN, stdin);
	// Remove trailing newline
	for (int i = 0; input_filename[i] && i < (signed)BUF_LEN; i++) {
	    if (input_filename[i] == '\r' || input_filename[i] == '\n') {
		input_filename[i] = '\0';
		break;
	    }
	}
    } else {
        input_filename = argv[1];
    }

    // Open input file and create an output file with an appropriate name
    script_fopen(input_filename);
    char *output_filename = malloc((strlen(input_filename) + 5) * sizeof(char));
    strcpy(output_filename, input_filename);
    strcat(output_filename, ".txt");
    output = fopen(output_filename, "w");
    if (!output) {
	puts("ERROR: output file could not be opened, aborting");
	return 1;
    }
    free(output_filename);
    
    // Free the buffer if we had to allocate it earlier
    if (allocated) {
        free(input_filename);
    }
    
    // Print file header
    fprintf(output, "#Time    Frequency    Polarization*100    Dose    Polarization_rate*100    Optimal_freq_positive    Direction   k_val\n");

    // Start serial
    int read = script_readline();
    // Check whether serial should be on or off
    if (script_cmdequ("serial")) {
	if (!strcmp(script_getarg(0), "on")) {
	    puts("Serial communications on");
	    serial_on = true;
	} else if (!strcmp(script_getarg(0), "off")) {
	    puts("Serial communications off");
	    serial_on = false;
	}

	read = script_readline(); // Get next line ready
    }

    if (serial_on) {
	start_serial(PORT);
    }
    
    do {
        if (script_cmdequ("init")) {
            // Initializer block
            if (did_init) {
                puts("ERROR: Cannot have more than one initializer block");
                puts("Aborting");
                return 1;
            }
            if (in_init) {
                goto INVALID_COMMAND;
            }
            in_init = true;
        } else if (script_cmdequ("done")) {
            // End initializer block
            if (!in_init) {
                goto INVALID_COMMAND;
            }
            
            in_init = false;
            did_init = true;
        } else if (script_cmdequ("mfld")) {
            if (!in_init) {
                goto INVALID_COMMAND;
            }

            sscanf(script_getarg(0), "%lf", &field);
            printf("Field set to %lf T\n", field);
        } else if (script_cmdequ("sdst")) {
            // Set steady state
            if (!in_init) {
                goto INVALID_COMMAND;
            }
            
            sscanf(script_getarg(0), "%lf", &max_steady_state);
            printf("Setting steady state: %lf\n", max_steady_state);
            steady_state = max_steady_state;
        } else if (script_cmdequ("temp")) {
            if (!in_init) {
                goto INVALID_COMMAND;
            }

            sscanf(script_getarg(0), "%lf", &temp);
            printf("Temperature set to %lf K\n", temp);
            reset_steady_state();
        } else if (script_cmdequ("freq")) {
            // Change frequency
            sscanf(script_getarg(0), "%lf", &freq);
            printf("Change frequency: %lf GHz\n", freq);
        } else if (script_cmdequ("time")) {
            // Run until time
            if (in_init) {
                goto INVALID_COMMAND;
            }
            
            double in_time;
            sscanf(script_getarg(0), "%lf", &in_time);
            // Use + to designate relative time (rather than absolute)
            if (script_getarg(0)[0] == '+') {
                in_time += sim_time;
            }
            printf("Running until time %lf\n", in_time);
            run_until(in_time);
        } else if (script_cmdequ("beam")) {
            // Beam on or off
            if (strcmp(script_getarg(0), "on") == 0) {
                puts("Turning beam on");
                beam_on(MAX_DOSE_RATE);
            } else if (strcmp(script_getarg(0), "off") == 0) {
                puts("Turning beam off");
                beam_off();
            } else {
                goto INVALID_COMMAND;
            }
        } else if (script_cmdequ("trip")) {
	    double in_time;
	    sscanf(script_getarg(0), "%lf", &in_time);
	    printf("Simulating beam trip: %lf s\n", in_time);

	    // Do a beam trip
	    beam_off();
	    tripping = true;
	    old_steady_state = steady_state;
	    steady_state *= 1.2;
	    max_pol_rate *= 10;
	    if (steady_state > max_steady_state) {
		steady_state = max_steady_state;
	    }

	    // Simulate for <time> seconds
	    run_until(sim_time + in_time/2);

	    // Turn the beam back on and keep tripping
	    beam_on(MAX_DOSE_RATE);
	    steady_state = old_steady_state;
	    max_pol_rate /= 10;

	    run_until(sim_time + in_time/2);
	    tripping = false; // End beam trip
	} else if (script_cmdequ("annl")) {
            double annl_time = atof(script_getarg(0));
            double annl_temp = atof(script_getarg(1));
            
            printf("Annealing for %lf s at %lf K\n", annl_time, annl_temp);
            anneal(annl_time, annl_temp);
        } else if (script_cmdequ("rand")) {
            // Turn randomness on/off
            if (!in_init) {
                goto INVALID_COMMAND;
            }
            
            if (strcmp(script_getarg(0), "on") == 0) {
                puts("Thermal fluctuations enabled");
                randomness_on = true;
            } else if (strcmp(script_getarg(0), "off") == 0) {
                puts("Thermal fluctuations disabled");
                randomness_on = false;
            } else {
                goto INVALID_COMMAND;
            }
        } else if (script_cmdequ("fllw")) {
	    if (serial_on) {
		puts("Can't follow frequency when serial is enabled!");
	    } else if (strcmp(script_getarg(0), "on") == 0) {
		puts("Following ideal frequency");
		follow_freq = true;
	    } else if (strcmp(script_getarg(0), "off") == 0) {
		puts("Not following ideal frequency");
		follow_freq = false;
	    }
	} else {
            INVALID_COMMAND:
            printf("Invalid command: %s\n", script_getarg(-1));
        }
	
    } while ((read = script_readline()));
    
    script_fclose();
    fclose(output);

    printf("Press enter to exit...");
    getchar();
}

double optimal_freq_pos() {
    //return (140.15 - 0.0125 * dose) * 5.0 / field;
    
    //return (140.2 - 0.0175 * dose) * 5.0 / field;
    //"The positive polarization frequencies are more linear as they drift lower, from about 140.20 to near 140.13 GHZ in SANE."
    //From Polarized Sources, Targets and Polarimetry...Proceedings of the 13th Inernational Workshop. Pg. 151
    
    //Update 10/14/2015:
    //here is a curve for optimal POS freq based on SANE data 
    double A_pos = 140.1; //This is the "steady state" frequency
    double C_pos = 0.045; //This is the range; add this to A to get the initial frequency
    double k_pos = 0.38;  //This determines the decay rate
    
    return (A_pos + C_pos*exp(-k_pos*dose))*5.0/field;
}


double optimal_freq_neg() {
    //return (140.45 + 0.025 * dose) * 5.0 / field;
    
    //return(140.4 + 0.0325 * dose) * 5.0 / field;
    //"In the case of DNP for negative polarization...a fast increase in the optimum microwave frequency which quickly slows, 
    //creating an exponential curve which...goes from 140.4 to around 150.53 GHz at the end of the anneal cycle (close to 4 Pe/cm^2)"
    //--same source as "optimal_freq_pos()"
    
    //Update 10/14/2015:
    //here is a curve for optimal NEG freq based on SANE data 
    double A_neg = 140.535; //This is the "steady state" frequency
    double C_neg = 0.065; //The range, subtract this from A to get the initial frequency
    double k_neg = 3.8; //This determines growth rate
    
    return (A_neg - C_neg*exp(-k_neg*dose))*5.0/field;
}


double deviation_increasing(double freq_diff) {
    return 1 / (1 + 30000.0 * freq_diff * freq_diff) - 0.05;
    //return 0.95 / (1 + 900.0 * freq_diff * freq_diff) + 0.05;
}

double deviation_decreasing(double freq_diff) {
    return 1 / (1 + 30000.0 * (freq_diff-0.025) * (freq_diff-0.025)) - 0.05;
}

void update_steady_state(double delta_t) {
    const double delta_dose = delta_t * dose_rate;
    // Choose the proper critical dose value (out of the three possible)
    // Critical Dose Source: "Proceedings of 4th International Workshop on Polarized Target Materials and Techniques" pg. 26
    double crit_dose;
    if (dose - last_anneal_dose > CDOSE_THRESHOLD[2]) {
        crit_dose = critical_dose[2];
    } else if (dose - last_anneal_dose > CDOSE_THRESHOLD[1]) {
        crit_dose = critical_dose[1];
    } else {
        crit_dose = critical_dose[0];
    }
    
    // Calculate some deviation factor based on the beam
    /*double ideal1, ideal2;
    if (freq > POS_NEG_DIFFERENTIATOR) {
        // Polarizing negatively
        ideal2 = optimal_freq_neg();
        dose -= delta_dose;
        ideal1 = optimal_freq_neg();
        dose += delta_dose;
    } else {
        // Polarizing positively
        ideal2 = optimal_freq_pos();
        dose -= delta_dose;
        ideal1 = optimal_freq_pos();
        dose += delta_dose;
    }
    double dev1 = deviation(freq - ideal1) + 0.025;
    double dev2 = deviation(freq - ideal2) + 0.025;
    double dev_quotient = dev1 / dev2;
    if (dev_quotient < 1) {
        steady_state *= dev_quotient;
    }*/

    //double old_steady_state = steady_state;
    steady_state *= exp(-delta_dose / crit_dose);
    //printf("%lf %lf %lf\n", steady_state, dev1, dev2);
}

void reset_steady_state() {
    // Yields 95% at 1K and 72% at 1.62K (from "Polarization Studies with Radiation Doped Ammonia at 5T and 1K*", (1990), fig. 14) 
    steady_state = max_steady_state*exp(-0.4471*(temp - 1));
    if (steady_state > 1.0) {
        steady_state = 1.0;
    }
}

double get_steady_state(double deviation) {
    return steady_state - 0.05*(0.95 - fabs(deviation))/0.95;
}

void update_pol(double delta_t) {
    
    update_steady_state(delta_t);  //"initialize" the steady state value, from which all the following calculations are made
    
    //NOTE: This function is based on exponential growth and decay functions (y= A +/- C*exp(-kx))  [+C for decay, -C for growth]
    //A = The "steady state" value you're ultimately trying to reach
    //C = the difference between A and where you are now
    //|k| = the rate of growth or decay

    double new_pol;
    double k_max = 0.0025;    //this value allows for max polarization in 20 minutes
    //double k_val;             //some percentage of k_max; based off of a Lorentzian curve  (aka the deviation_increasing/decreasing functions)
    double freq_range = 0.05; //GHz    (Based on SANE data)
    double percent_ideal_neg;
    double percent_ideal_pos;
      
    //For NEGATIVE Polarization:         
    if (freq > POS_NEG_DIFFERENTIATOR) {
	double ideal = optimal_freq_neg();
            
        // Follow frequency if turned on
        if (follow_freq) {
            freq = optimal_freq_neg();
        }
        percent_ideal_neg = 1 - ((fabs(ideal-freq)))/(freq_range);
        if (percent_ideal_neg >= 0.500){
            double dev = deviation_increasing(ideal - freq);
            k_val = k_max * dev;
            new_pol = -(get_steady_state(dev) - (get_steady_state(dev) - pol)*exp(-k_val * delta_t));
        }
        else {
            k_val = k_max * (1-deviation_decreasing(ideal - freq));
            if (k_val > k_max) {
            k_val = k_max;
            }
            new_pol = -(0 + pol*exp(-k_val*delta_t));  
        }
    } 
    


    //For POSITIVE Polarization:
    else {
        double ideal = optimal_freq_pos();
        if (follow_freq) {
            freq = optimal_freq_pos();
        }
        percent_ideal_pos = 1 - ((fabs(ideal-freq)))/(freq_range);  //essentially, how far away are you from the ideal freq. 
        if (percent_ideal_pos >= 0.500){                            //if you are within 50% of the specified range; polarization rate will be positive
            double dev = deviation_increasing(ideal - freq);
            k_val = k_max * dev;
            /*if (pol >= steady_state) {
                k_val = k_max - k_val;
            }*/
            new_pol = get_steady_state(dev) - fabs(get_steady_state(dev) - pol)*exp(-k_val * delta_t);
            //if (new_pol > steady_state){
              //    new_pol = steady_state - (pol-steady_state)*exp(-k_val * delta_t);
            //    k_val = k_max - k_val;
            //    new_pol = steady_state - (steady_state - pol )*exp(-k_val * delta_t);
            //}      
        }
        else {                                                      //if you are not within 50% of the specified range; polarization rate will be negative (decreasing pol)
            k_val = k_max * (1-deviation_decreasing(ideal - freq));
            if (k_val > k_max) {
            k_val = k_max;                                         //polarization decay rate cannot be bigger than its growth rate
            }
            new_pol = 0 + pol*exp(-k_val*delta_t);  
        }
    } 
    
    pol = new_pol;
    dose += dose_rate * delta_t;
    
}

void update() {
    // Output data at this step (as long as we're not in serial mode)
    // In serial mode, the data should be output when a new set of values
    // is provided by the box (see the process_command function)
    if (!serial_on) {
	output_data();
    }
    
    double old_pol = pol;
    sim_time += DELTA_T;
    for (int i = 0; i < N_ITER; i++) {
	update_pol(DELTA_T / N_ITER);
    }

    // Calculate pol_rate if the serial cannot provide it
    if (!serial_on) {
	pol_rate = (pol - old_pol) / DELTA_T;
    }

    // Thermal fluctuations (if enabled)
    if (randomness_on) {
        double percent = rand_int(0,BASE_RANDOMNESS + (int)(1000000*dose_rate)) / 1000000.;
        double newpol;
        if (rand_int(0, 2) == 0) {
            newpol = pol + pol * percent;
        } else {
            newpol = pol - pol * percent;
        }
        if (fabs(newpol) < (1 + percent) * max_steady_state) {
            pol = newpol;
        }
    }
}

void run_until(double new_time) {
    time_t old_time;
    time(&old_time);
    
    while (sim_time < new_time) {
        time_t curr_time;
        time(&curr_time);
        
        // Process input commands if there are any
	if (serial_on) {
	    process_command();
	}
        
        // Wait until DELAY before updating (if serial is on)
	if (serial_on) {
	    if (difftime(curr_time, old_time) >= DELAY) {
		update();
		printf("Simulation time: %lf\n", sim_time);
            
		old_time = curr_time;
	    }
	} else {
	    update();
	}
    }
}

void beam_on(double rate) {
    dose_rate = rate;
}

void beam_off() {
    dose_rate = 0.0;
}

void anneal(double annl_time, double temp) {
    temp++; // suppress unused parameter warning :)
    double time_lim = annl_time + sim_time;
    double old_pol = pol;
    pol = 0;

    for (; sim_time < time_lim; sim_time += DELTA_T) {
	// If there's no serial, we should output the data during the anneal
	// here because we don't have to worry about getting pol_rate from the box
	// (we just need to calculate it ourselves)
	if (!serial_on) {
	    pol_rate = 0;
	    output_data();
	}
    }
    pol = old_pol;
    
    last_anneal_dose = dose;
    reset_steady_state();
}

void output_data() {
    // It's super annoying to have these diagnostic messages when there's
    // no serial communications to delay them
    if (serial_on) {
	puts("Writing to file");
    }
    fprintf(output, "%lf %lf %lf %lf %lf %lf %lf %lf\n", sim_time, freq, 100*pol, dose, 100*pol_rate, optimal_freq_pos(), direction, k_val);
    fflush(output);
}

int rand_int(int min, int max) {
    return (int)((max-min)*(double)rand()/RAND_MAX) + min;
}

void delay(int seconds) {
    time_t start, end;
    time(&start);
    
    do {
        time(&end);
    } while (difftime(end, start) < seconds);
}

void process_command() {
    uint8_t control = rxByte(PORT);
    while (control) {
        switch((int)control) {
        case 0x11:
            puts("Reading frequency");
            readFreq();
            break;
        case 0x33:
            puts("Confirmation requested");
            sendConfirmation();
            break;
        case 0x77:
            puts("Writing event number");
            writeEventNum();
            break;
        case 0xBB:
            puts("Reading polarization rate");
            readPolRate();
            break;
        case 0x88:
            puts("Reading motor direction");
            readDirection();
	    // The polarization rate is the last bit of data to be
	    // sent by the box, so we know we have a complete
	    // row of data to output at this point
            output_data();
            break;
        case 0xFF:
            puts("Writing polarization");
            writePol();
            break;
        }
        
        control = rxByte(PORT);
    }
}

// Sends confirmation code
void sendConfirmation() {
    txByte(PORT, 0xBE);
    txByte(PORT, 0xEF);
}

// Reads a polarization rate from the Propeller
void readPolRate() {
    pol_rate = (double)readFloat(PORT);
}

// Reads frequency updates from Propeller
void readFreq() {
    int32_t freqInt = readInt32(PORT);
    freq = (double)freqInt / 1000.0;
}

//Reads motor direction from the Propeller
void readDirection(){
    direction = (double)readInt32(PORT);
}

// Output frequency to Propeller
void writeFreq() {
    // Output frequency in MHz as integer
    writeInt32(PORT, (int32_t)(freq * 1000));
}

// Outputs polarization to Propeller controller
void writePol() {
    // As a float
    writeFloat(PORT, pol);
}

// Output eventnum to Propeller
void writeEventNum() {
    time_t eventTime;
    time(&eventTime);
    uint32_t eventNum = (uint32_t)eventTime;

    writeInt32(PORT, eventNum);
}
