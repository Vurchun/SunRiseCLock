#include <iostream>
#include <math.h> // для функції pow




double fiftyApr(double i){
    return 0.00000020558737239186*i*i*i + 0.00008493495625480629*i*i - 0.00056135584043426334*i + 0.00220175565991809208;
}


double hungredApr(double i){
    return 0.00000000000020774765*i*i*i + 0.00000000845953934651*i*i - 0.00000545909469979247*i+ 0.00201593222599072419;
}

double thouthandApr(double i){

    return 0.00000000020754957593*i*i*i + 0.00000084626309852429*i*i - 0.00005473036906744611*i + 0.00203250994804093921;
}


int main() {
    // Масив значень x, для яких будемо обчислювати x^2.24

    // Степінь, до якої будемо підносити
    double exponent = 2.24;
    double fifty[10000]={};
    double hungred[10000]= {};
    double thouthand[10000]={};
    double fiftyDeviation = 1.0;
    double hungredDeviation = 1.0;
    double thouthandDeviation = 1.0;

    for (double i = 0.0001; i < 1; i+=0.0001) {
         fifty[(int)(i*10000)]        = pow((pow(i, exponent) - fiftyApr(i)),2);
         hungred[(int)(i*10000)]      = pow((pow(i, exponent) - hungredApr(i)),2);
         thouthand[(int)(i*10000)]    = pow((pow(i, exponent) - thouthandApr(i)),2);
    }
    for (int i = 1; i < 10000; i++) {
    fiftyDeviation      += fifty[i];
    hungredDeviation    += hungred[i];
    thouthandDeviation  += thouthand[i];
    }

    std::cout <<"fiftyDeviation = "<< pow(fiftyDeviation/10000,0.5) << std::endl;
    std::cout <<"hungredDeviation = "<< pow(hungredDeviation/10000,0.5) << std::endl;
    std::cout <<"thouthandDeviation = "<< pow(thouthandDeviation/10000,0.5) << std::endl;


    return 0;
}
