#include <stdio.h>

int main(){
    int num = 1024;

    while(num != 0){
        int digit = num % 10;
        num = num / 10;
        printf("%d\n", digit);
    }

    return 0;
}
