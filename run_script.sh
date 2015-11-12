echo Are the nodes mobile?
read mobility
echo Enter number of packets
read noPackets
echo Enter number of interference packets
read inoPackets
echo Enter sifs value
read sifs

echo ------------------------------------------------------- > results

den=6
while [ $den -le 120 ]
do
	echo ------------------------------------------------------- >> results
	echo noPackets $noPackets inoPackets $inoPackets sifs $sifs vehicle_density $den >> results
	echo ------------------------------------------------------- >> results
	for i in {1..100}
	do 
		./waf --run ./waf --run "scratch/dsrc --nodeMobility=$mobility --noPackets=$noPackets --inoPackets=$inoPackets --sifs=$sifs --isifs=$i --vehicle_density=$den" | grep PDR > temp_results; 
		echo $i >> results
		cat temp_results >> results 
	done
	let "den += 6"
	echo $den
done
#for i in {1..100}; do ./waf --run ./waf --run "scratch/dsrc --nodeMobility=$mobility --noPackets=$noPackets --inoPackets=$inoPackets --sifs=$sifs --isifs=$i --vehicle_density=$vehicle_density" ; done
