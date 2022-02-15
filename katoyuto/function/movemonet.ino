double movemonet(double array[]){
    int n = sizeof(array);
    double new_array[n]
    for(int i=1; i<n; i++){
        new_array[i - 1] = array[i];
    }
    return new_array;
}

double ar_ave(double array[]){
    int n = sizeof(array);
    double sum = array[0]
    for( i=1;i<n;++i ){
    sum += array[i];
    }
    return sum/n;
}