#include <vector>
#include<bits/stdc++.h>

using namespace std;

struct Interval{
    double s;
    double e;
};

vector<Interval> union_interval(vector<Interval>& ins){
    if(ins.empty()) return vector<Interval>{};
    vector<Interval> res;
    sort(ins.begin(), ins.end(), [](Interval a, Interval b){return a.s < b.s;});

    res.push_back(ins[0]);
    for(int i = 1; i < ins.size(); i++) {
        if (res.back().e < ins[i].s) res.push_back(ins[i]);
        else
            res.back().e = max(res.back().e, ins[i].e);
    }
    return res;
}

Interval intersect_interval(vector<Interval>& ins){
    if(ins.empty()) return Interval{};
    Interval res;
    sort(ins.begin(), ins.end(), [](Interval a, Interval b){return a.s < b.s;});
    res.s = ins.back().s;
    sort(ins.begin(), ins.end(), [](Interval a, Interval b){return a.e < b.e;});
    res.e = ins.front().e;

    if(res.s > res.e) return Interval{};
    return res;
}

int main(){
    vector<Interval> arr = {{2,3},{1,1.3},{2.5,7},{0,0.5},{5,7},{8.9,10}}, arr2 = {{1,5},{2,6},{3,7}};
    vector<Interval> arr_merge;
    Interval arr_inter;

    arr_merge = union_interval(arr);
    arr_inter = intersect_interval(arr2);

    cout << "Merged Interval: ";
    for(int i=0; i<arr_merge.size(); i++){
        cout << '[' << arr_merge[i].s << ',' << arr_merge[i].e << ']';
    }

    cout << "Intersect Interval: " << '[' << arr_inter.s << ',' << arr_inter.e << ']';

    return 0;
}
