#include "include/Interval.h"

std::vector<Interval> Interval::unions(std::vector<Interval> &ins) {
    // Union of several intervals
    if (ins.empty()) {
        return std::vector<Interval>{};
    }
    std::vector<Interval> res;
    sort(ins.begin(), ins.end(),
         [](Interval a, Interval b) { return a.s < b.s; });

    res.push_back(ins[0]);
    for (size_t i = 1; i < ins.size(); i++) {
        if (res.back().e < ins[i].s) {
            res.push_back(ins[i]);
        } else {
            res.back().e = std::max(res.back().e, ins[i].e);
        }
    }
    return res;
}

std::vector<Interval> Interval::intersects(std::vector<Interval> &ins) {
    // Intersection of several intervals
    if (ins.empty()) {
        return std::vector<Interval>{};
    }
    std::vector<Interval> res;
    Interval buff;

    sort(ins.begin(), ins.end(),
         [](Interval a, Interval b) { return a.s < b.s; });
    buff.s = ins.back().s;
    sort(ins.begin(), ins.end(),
         [](Interval a, Interval b) { return a.e < b.e; });
    buff.e = ins.front().e;

    if (buff.s > buff.e) {
        return std::vector<Interval>{};
    }
    res.push_back(buff);
    return res;
}

std::vector<Interval> Interval::complements(std::vector<Interval> &outer,
                                            std::vector<Interval> &inner) {
    // Complement between one outer interval and several inner intervals
    if (outer.empty()) {
        return std::vector<Interval>{};
    }
    if (inner.empty()) {
        return outer;
    }

    std::vector<Interval> res, comp, int_buff, intsect;
    sort(inner.begin(), inner.end(),
         [](Interval a, Interval b) { return a.s < b.s; });

    // Compliment of the inner intervals
    comp.push_back({-std::numeric_limits<double>::max(), inner[0].s});
    for (size_t i = 0; i < inner.size() - 1; i++) {
        comp.push_back({inner[i].e, inner[i + 1].s});
    }
    comp.push_back({inner.back().e, std::numeric_limits<double>::max()});

    // Intersection with outer interval
    for (size_t i = 0; i < comp.size(); i++) {
        int_buff.push_back(comp[i]);
        int_buff.push_back(outer[0]);

        intsect = intersects(int_buff);
        if (!intsect.empty()) res.push_back(intsect[0]);

        int_buff.clear();
    }

    return res;
}
