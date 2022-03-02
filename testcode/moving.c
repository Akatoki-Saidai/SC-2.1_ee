#include <stdio.h>

double new_array[9];
double *p = new_array;

double *movement(double array[], int size){

    for(int i=1; i<size; i++){
        new_array[i - 1] = array[i];
    }
    return new_array;
}

/*double ar_ave(double array[]){
    int n = sizeof(array);
    double sum = array[0];
    for( i=1;i<n;++i ){
        sum += array[i];
    }
    return sum/n;
}*/


int main(void){
    double array[9];
    array[0]=234;
    array[1]=345;   
    array[2]=456;    
    array[3]=567;   
    array[4]=890;
    array[5]=1000;
    array[6]=2000;
    array[7]=3000;
    array[8]=4000;
    double update=5000;
    //double azimuth;
    int n = sizeof(array)/sizeof(double);
    int i;
    p = movement(array,n);
    
    for(i=1;i<n;i++){
       array[i-1] = *p;
       p++;
    }
    
    array[i-1] = update;
    
    for(i=0;i<n;i++){
       printf("%lf\n",array[i]);
    }

    //printf("%lf\n",azimuth);

    return 0;
    
}