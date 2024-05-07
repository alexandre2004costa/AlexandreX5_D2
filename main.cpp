
#include "DisplayMenu.h"
#include "Menu.h"
#include <chrono>

int main() {
    Menu menu;
    double distance = menu.haversineDistance(52.2296756, 21.0122287, 48.8567, 2.3508);
    cout<<distance<<endl;
   /* DisplayMenu displayMenu = DisplayMenu();
    displayMenu.OpenMenu();*/

    return 0;
}
