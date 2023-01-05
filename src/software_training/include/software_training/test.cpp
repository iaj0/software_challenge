#include <vector>
#include <iostream>

class bro {
    public:
        int ret() {
            return net[0].x;
        }
    private:
        typedef struct test {
            test(int x, int y) : x{x}, y{y} {}
            int x;
            int y;
        } bra;

        std::vector<bra> net {{1,2}};

    
};



int main () {
    bro my;

    std::cout<<my.ret();

    return 0;
}