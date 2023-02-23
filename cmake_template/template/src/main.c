#include <iostream>
#include <stdio.h>
using namespace std;
// #define Inline __inline__ __attribute__((always_inline))

//声明内联函数
void swap1(int *a, int *b);  //也可以添加inline，但编译器会忽略
static inline int getMax(int left, int right);
int main(){
    int m, n;
    cin>>m>>n;
    cout<<m<<", "<<n<<endl;
    swap1(&m, &n);
    cout<<m<<", "<<n<<endl;
    // m=getMax(m,n);
    return 0;
}

static inline int getMax(int left, int right) {
        return left > right ? left : right;
}

//定义内联函数
inline void swap1(int *a, int *b){
    int temp;
    temp = *a;
    *a = *b;
    *b = temp;
}