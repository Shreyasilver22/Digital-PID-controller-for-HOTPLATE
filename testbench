*-----------------------------------------------------------------------------
// Module:    tb_pid
// Project:   Digital Temperature Control
//
// Description:
//   Testbench for the PID controller and hotplate.
//   - Connects the controller to the hotplate in a feedback loop.
//   - Generates a clock signal.
//   - Applies a 'setpoint' and monitors the system response.
//-----------------------------------------------------------------------------*/
module tb_pid;

    // --- Signals ---
    reg                       clk;
    reg                       reset;
    reg  signed [15:0]        setpoint;
    wire signed [15:0]        temp_from_plate;
    wire [7:0]                power_to_plate;

    // --- Instantiate the Controller ---
    // We can override the parameters here to "tune" the controller
    // without editing the original file.
    pid_controller #(
        .Kp(20),
        .Ki(2),
        .Kd(20)
    ) dut_pid (
        .clk(clk),
        .reset(reset),
        .setpoint_temp(setpoint),
        .current_temp(temp_from_plate),
        .heater_power(power_to_plate)
    );

    // --- Instantiate the Plant ---
    hotplate dut_plate (
        .clk(clk),
        .reset(reset),
        .heater_power(power_to_plate),
        .current_temp(temp_from_plate)
    );

    // --- Clock Generation (10ns period) ---
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // --- Simulation Stimulus ---
    initial begin
        // 1. Reset the system
        reset = 1;
        setpoint = 150; // Set our target to 150 degrees
        #20;            // Hold reset for 20ns
        reset = 0;

        // 2. Let it run for 1000ns to stabilize
        #1000;

        // 4. Change the setpoint back down to 100
        //$display("--- Changing setpoint to 100 ---");
        //setpoint = 100;
        //#1000;

        // 5. End simulation
        $display("--- Simulation complete ---");
        $stop;
    end
