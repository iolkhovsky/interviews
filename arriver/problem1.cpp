#include <iostream>

int main(int argc, char** argv) {
    int N;
    std::cin >> N;
    int lr_slats = 0;
    int tb_slats = 0;
    for (int i = 0; i < N; i++) {
        std::string s;
        std::cin >> s;
        for (int j = 0; j < 2; j++)
            tb_slats += s[j] == '0';
        for (int j = 2; j < 4; j++)
            lr_slats += s[j] == '0';
    }
    int out = std::min(tb_slats, lr_slats) / 2;
    std::cout << out << " " << tb_slats - out * 2 << " " << lr_slats - out * 2;
    return 0;
}
