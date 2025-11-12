/*-----------------------------------------------------------------------------
// Module:    hotplate
// Project:   Digital Temperature Control
//
// Description:
//   A simple "plant" model for a hotplate.
//   - Heats up based on 'heater_power' input.
//   - Loses a constant amount of heat ('COOLING_RATE') every cycle
//     to simulate cooling to the air.
//   - Has a minimum ('AMBIENT_TEMP') and maximum temp.
//-----------------------------------------------------------------------------*/
module hotplate (
    input                       clk,
    input                       reset,
    input      [7:0]            heater_power,
    output reg signed [15:0]    current_temp
);

    // --- Model Parameters ---
    // How much the temp changes each cycle from cooling
    parameter COOLING_RATE = 2;
    // How much 'heater_power' affects temperature.
    // (A shift of 4 is like dividing by 16)
    parameter HEATING_SHIFT = 3;
    // The temperature of the room
    parameter AMBIENT_TEMP = 25;
    // A safety cutoff
    parameter MAX_TEMP = 400;

    // --- Internal variable for calculation ---
    reg signed [15:0] temp_change;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            // Start at room temperature
            current_temp <= AMBIENT_TEMP;
        end
        else begin
            // 1. Calculate the change in temperature for this cycle
            //    temp_change = (Heat In) - (Heat Out)
            temp_change = (heater_power >> HEATING_SHIFT) - COOLING_RATE;

            // 2. Apply the change and saturate (clamp) the temperature
            if (current_temp + temp_change < AMBIENT_TEMP) begin
                // Don't cool below room temperature
                current_temp <= AMBIENT_TEMP;
            end
            else if (current_temp + temp_change > MAX_TEMP) begin
                // Don't heat above the max
                current_temp <= MAX_TEMP;
            end
            else begin
                // This is the normal operating case
                current_temp <= current_temp + temp_change;
            end
        end
    end

endmodule
