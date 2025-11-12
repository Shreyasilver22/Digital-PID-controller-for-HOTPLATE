/*-----------------------------------------------------------------------------
// Module:    pid_controller
// Project:   Digital Temperature Control
//
// Description:
//   A digital PID controller implemented in Verilog.
//   - Reads a 'setpoint_temp' and 'current_temp'.
//   - Calculates Proportional, Integral, and Derivative terms.
//   - Outputs an 8-bit 'heater_power' signal (0-255).
//   - Includes integral "anti-windup" to prevent instability.
//-----------------------------------------------------------------------------*/
module pid_controller (
    input                       clk,
    input                       reset,
    input      signed [15:0]    setpoint_temp,
    input      signed [15:0]    current_temp,
    output reg [7:0]            heater_power
);

    // --- PID Gain Parameters ---
    // These are the "tuning knobs" for your controller.
    // You will change these values in the testbench to see how they
    // affect the system's response (overshoot, stability, etc.)
    parameter Kp = 20;  // Proportional Gain
    parameter Ki = 2;   // Integral Gain
    parameter Kd = 20;   // Derivative Gain

    // --- Scaling Parameter ---
    // The PID calculation results in a large number. We need to scale
    // it down to fit our 8-bit (0-255) output.
    // A shift of 8 is like dividing by 2^8 = 256.
    parameter OUTPUT_SHIFT = 8;

    // --- Internal Registers ---
    reg signed [15:0] error;
    reg signed [15:0] last_error;
    reg signed [31:0] integral_sum; // Needs to be wide to prevent overflow

    // --- Term Registers (for clarity) ---
    reg signed [31:0] p_term;
    reg signed [31:0] i_term;
    reg signed [31:0] d_term;

    // --- Raw PID Output (before scaling) ---
    reg signed [31:0] pid_sum_raw;

    // --- Main PID Logic Block ---
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            // Reset all internal states
            error         <= 0;
            last_error    <= 0;
            integral_sum  <= 0;
            p_term        <= 0;
            i_term        <= 0;
            d_term        <= 0;
            pid_sum_raw   <= 0;
            heater_power  <= 0;
        end
        else begin
            // 1. Calculate Error
            error <= setpoint_temp - current_temp;

            // 2. Calculate P Term
            p_term <= Kp * error;

            // 3. Calculate I Term (with Anti-Windup)
            //    "Anti-Windup" is critical: If the heater is already at
            //    100% (255) and the temp is still too low, we must
            //    STOP accumulating the integral. Otherwise, 'integral_sum'
            //    will grow to a massive value, causing a huge overshoot later.
            if (heater_power == 255 && error > 0) begin
                // Don't accumulate more positive error (we're maxed out)
                integral_sum <= integral_sum;
            end
            else if (heater_power == 0 && error < 0) begin
                // Don't accumulate more negative error (we're off)
                integral_sum <= integral_sum;
            end
            else begin
                // Normal operation: accumulate the error
                integral_sum <= integral_sum + error;
            end
            i_term <= Ki * integral_sum;

            // 4. Calculate D Term
            //    This is the "brake" - it measures the *change* in error.
            d_term <= Kd * (error - last_error);
            last_error <= error; // Save current error for next cycle

            // 5. Calculate Final Raw Sum
            pid_sum_raw <= p_term + i_term + d_term;

            // 6. Scale and Saturate Output
            //    Convert the large 'pid_sum_raw' to our 8-bit power signal.
            if (pid_sum_raw > (255 << OUTPUT_SHIFT)) begin
                // Clamp at max power
                heater_power <= 255;
            end
            else if (pid_sum_raw < 0) begin
                // Clamp at min power
                heater_power <= 0;
            end
            else begin
                // Scale the result down to 0-255
                heater_power <= pid_sum_raw >> OUTPUT_SHIFT;
            end
        end
    end

endmodule
