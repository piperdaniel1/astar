#include <vector>
#include <iostream>
#include <algorithm>

using namespace std;

int main() {
    vector<int> v = {1, 2, 3, 4, 5};

    for (auto i : v) {
        cout << i << endl;
    }

    sort(v.begin(), v.end(), [](int a, int b) { return a > b; });

    for (auto i : v) {
        cout << i << endl;
    }
    return 0;
}