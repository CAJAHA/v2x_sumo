for infVeh in 0 5 10 15 20
do
    for run in 1 2 3 4 5
    do
        c="v2x_communication_example --infVeh=${infVeh} --run=${run}"
        echo $c
        ./waf --run "$c"
    done
done