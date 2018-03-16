#include<vector>
#include<bits/stdc++.h>

using namespace std;

struct Interval{
    double s;
    double e;
};

vector<Interval> Union(vector<Interval>& ins){
    // Union of several intervals
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

vector<Interval> Intersect(vector<Interval>& ins){
    // Intersection of several intervals
    if(ins.empty()) return vector<Interval> {};
    vector<Interval> res;
    Interval buff;

    sort(ins.begin(), ins.end(), [](Interval a, Interval b){return a.s < b.s;});
    buff.s = ins.back().s;
    sort(ins.begin(), ins.end(), [](Interval a, Interval b){return a.e < b.e;});
    buff.e = ins.front().e;

    if(buff.s > buff.e) return vector<Interval> {};
    res.push_back(buff);
    return res;
}

vector<Interval> Complement(vector<Interval>& outer, vector<Interval>& inner){
    // Complement between one outer interval and several inner intervals
    if(outer.empty()) return vector<Interval>{};
    if(inner.empty()) return outer;

    vector<Interval> res;
    sort(inner.begin(), inner.end(), [](Interval a, Interval b){return a.s < b.s;});

    for(int i=0; i<inner.size(); i++){
        if(outer[0].s <= inner.front().s) res.push_back({outer[0].s,inner.front().s});
    res.push_back({inner.back().e,outer[0].e});
    }

    for(int i=0; i<inner.size()-1; i++){
        res.push_back({inner[i].e,inner[i+1].s});
    }

    return Union(res);
}

int main(){
    cout << "max" << endl;
    vector<Interval> arr = {{2,3},{1,1.3},{2.5,7},{5,7},{8.9,10}}, arr2 = {{0,15},{0.3,13},{1,12}};
    cout << 'k' << endl;
    vector<Interval> arr_merge, arr_inter, arr_comp;

    cout << 'k' << endl;

    arr_merge = Union(arr);
    arr_inter = Intersect(arr2);
    arr_comp = Complement(arr_inter,arr_merge);

    if(!arr_merge.empty()){
        cout << "Merged Interval: ";
        for(int i=0; i<arr_merge.size(); i++){
            cout << '[' << arr_merge[i].s << ',' << arr_merge[i].e << ']';
        }
    }

    if(!arr_inter.empty()){
        cout << "Intersect Interval: ";
        for(int i=0; i<arr_inter.size(); i++){
            cout << '[' << arr_inter[i].s << ',' << arr_inter[i].e << ']';
        }
    }

    if(!arr_comp.empty()){
        cout << "Complement Interval: ";
        for(int i=0; i<arr_comp.size(); i++){
        cout << '[' << arr_comp[i].s << ',' << arr_comp[i].e << ']';
        }
    }

    return 0;
}
