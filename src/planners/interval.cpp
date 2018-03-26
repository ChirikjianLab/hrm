#include<vector>
#include<bits/stdc++.h>
#include<limits>

#include<src/planners/interval.h>

vector<Interval> interval::Union(vector<Interval>& ins){
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

vector<Interval> interval::Intersect(vector<Interval>& ins){
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

vector<Interval> interval::Complement(vector<Interval>& outer, vector<Interval>& inner){
    // Complement between one outer interval and several inner intervals
    if(outer.empty()) return vector<Interval>{};
    if(inner.empty()) return outer;

    vector<Interval> res, comp, int_buff, intsect;
    sort(inner.begin(), inner.end(), [](Interval a, Interval b){return a.s < b.s;});

    // Compliment of the inner intervals
    comp.push_back({-numeric_limits<double>::max(), inner[0].s});
    for(int i=0; i<inner.size()-1; i++){
        comp.push_back({inner[i].e,inner[i+1].s});
    }
    comp.push_back({inner.back().e, numeric_limits<double>::max()});

    // Intersection with outer interval
    for(int i=0; i<comp.size(); i++){
        int_buff.push_back(comp[i]);
        int_buff.push_back(outer[0]);

        intsect = Intersect(int_buff);
        if(!intsect.empty()) res.push_back(intsect[0]);

        int_buff.clear();
    }

    return res;
}

//int main(){
//    vector<Interval> arr = {{2,3},{1,1.3},{2.5,7},{5,7},{8.9,10}}, arr2 = {{0,15},{0.3,13},{6,12}};
//    vector<Interval> arr_merge, arr_inter, arr_comp;

//    interval op_interval;
//    arr_merge = op_interval.Union(arr);
//    arr_inter = op_interval.Intersect(arr2);
//    arr_comp = op_interval.Complement(arr_inter,arr_merge);

//    if(!arr_merge.empty()){
//        cout << "Merged Interval: ";
//        for(int i=0; i<arr_merge.size(); i++){
//            cout << '[' << arr_merge[i].s << ',' << arr_merge[i].e << ']';
//        }
//    }

//    if(!arr_inter.empty()){
//        cout << "Intersect Interval: ";
//        for(int i=0; i<arr_inter.size(); i++){
//            cout << '[' << arr_inter[i].s << ',' << arr_inter[i].e << ']';
//        }
//    }

//    if(!arr_comp.empty()){
//        cout << "Complement Interval: ";
//        for(int i=0; i<arr_comp.size(); i++){
//        cout << '[' << arr_comp[i].s << ',' << arr_comp[i].e << ']';
//        }
//    }

//    return 0;
//}
