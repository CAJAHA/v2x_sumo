for infVeh in 40
do
    for run in 444 555
    do
        c="v2x_communication_example --infVehNum=${infVeh} --run=${run}"
        echo $c
        ./waf --run "$c"
    done
done