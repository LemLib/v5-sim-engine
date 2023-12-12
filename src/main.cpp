#include <cstdio>
extern "C" void pros_init();
__attribute__((constructor(101))) void init() {
    pros_init();
}
int main() {

    for(;;);
    return 0;
}
