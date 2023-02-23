#include "test_class.h"
ValueClass::ValueClass(){
    printf("hello world \n");
}
void ValueClass::Add(int i, int j){
    sum = i+j;
    printf("sum : %d value : %d\n",sum,value);
}