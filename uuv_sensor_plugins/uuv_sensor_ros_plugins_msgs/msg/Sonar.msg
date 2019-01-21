# This message contains an uncompressed sonar beams

uint32 num_bins        # Number of bins of the sonar
uint32 num_beams          # Number of beams of the sonar

float32 beams_width          # Opening of the beam orthogonal to the device's Z direction

float32 beam_height          # Opening of the beam along the device's Z direction

float32 bearings       # This is the position of the beam center with respect to the "front" of the sonar (which is at 0)

float32[] data # actual data for the sonar normal