# Navigate to the directory where the test series results files get saved
cd "C:/Users/gamage_a/Documents/CM_Trail/SimOutput/WMGL241/20220708"
# Create a list of all the result files
set files [glob *.erg]
# Create a text file to record the results from all the tests runs in the test series
set outfile [open "combined.txt" a+]
## Set the precision size of the recorded data to 4 decimal points using global command
# set tcl_precision 4
# Loop through all the .erg files importing the relevant data channels to collate to the master results file
for {set f 0} {$f < [llength $files]} {incr f} {
    set fname [lindex $files $f]
    # Read the CM results file to get the array elements/ channels
    ImportResFile $fname {Car.tx Car.ty Car.Yaw Time} Results
    # Loop through the full length of each .erg file recording the channels required to be transferred to the master file to generate the training dataset
    for {set i 0} {$i < [llength $Results(Car.Yaw)]} {incr i} {
        puts $outfile "[lindex $Results(Time) $i]	[lindex $Results(Car.Fr1.tx) $i]	[lindex $Results(Car.Fr1.ty) $i]	[lindex $Results(Car.Road.Lane.Act.LaneId) $i]"
    }
}
# Close the master results file
close $outfile
