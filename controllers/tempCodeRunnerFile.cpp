#include <iostream>
#include <sstream>

// Fungsi untuk mengonversi warna hexa dalam format 0x55a473dd6670 menjadi RGB
void hexToRGB(uint64_t hexColor, int& red, int& green, int& blue) {
    // Ekstrak nilai warna RGB
    red = (hexColor >> 32) & 0xFF;
    green = (hexColor >> 16) & 0xFF;
    blue = hexColor & 0xFF;
}

int main() {
    // Contoh penggunaan
    uint64_t hexColor = 0x55a473dd6670; // Contoh warna
    int red, green, blue;

    hexToRGB(hexColor, red, green, blue);

    // Menampilkan hasil konversi
    std::cout << "Warna Hex: 0x" << std::hex << hexColor << std::dec << std::endl;
    std::cout << "Warna RGB: (" << red << ", " << green << ", " << blue << ")" << std::endl;

    return 0;
}
