#include "TestWrapper.h"
#include "test_class.h"
 
#ifdef __cplusplus
extern "C" {
#endif 
 
void myValueClass(int a, int b){
    ValueClass t;
    t.Add(a,b);
}
 
#ifdef __cplusplus
};
#endif 