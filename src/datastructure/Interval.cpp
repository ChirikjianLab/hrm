/** \author Sipu Ruan */

#include "hrm/datastructure/Interval.h"

hrm::Interval::Interval(const Coordinate start, const Coordinate end)
    : start_(start), end_(end) {}

hrm::Interval::Interval() = default;

std::vector<hrm::Interval> hrm::Interval::unions(
    const std::vector<Interval> &ins) {
    // Union of several intervals
    if (ins.empty()) {
        return std::vector<Interval>{};
    }

    auto insCopy = ins;
    std::vector<Interval> res;
    sort(insCopy.begin(), insCopy.end(),
         [](Interval a, Interval b) { return a.s() < b.s(); });

    res.push_back(insCopy.at(0));
    for (const auto element : insCopy) {
        if (res.back().e() < element.s()) {
            res.push_back(element);
        } else {
            res.back().setEnd(std::max(res.back().e(), element.e()));
        }
    }

    return res;
}

std::vector<hrm::Interval> hrm::Interval::intersects(
    const std::vector<Interval> &ins) {
    // Intersection of several intervals
    if (ins.empty()) {
        return std::vector<Interval>{};
    }

    std::vector<Interval> res;
    auto insCopy = ins;
    Interval buff;

    sort(insCopy.begin(), insCopy.end(),
         [](Interval a, Interval b) { return a.s() < b.s(); });
    buff.setStart(insCopy.back().s());
    sort(insCopy.begin(), insCopy.end(),
         [](Interval a, Interval b) { return a.e() < b.e(); });
    buff.setEnd(insCopy.front().e());

    if (buff.s() > buff.e()) {
        return std::vector<Interval>{};
    }
    res.push_back(buff);

    return res;
}

std::vector<hrm::Interval> hrm::Interval::complements(
    const std::vector<Interval> &outer, const std::vector<Interval> &inner) {
    // Complement between one outer interval and several inner intervals
    if (outer.empty()) {
        return std::vector<Interval>{};
    }
    if (inner.empty()) {
        return outer;
    }

    auto innerCopy = inner;
    std::vector<Interval> res;
    std::vector<Interval> comp;
    std::vector<Interval> intBuff;
    std::vector<Interval> intsect;

    sort(innerCopy.begin(), innerCopy.end(),
         [](Interval a, Interval b) { return a.s() < b.s(); });

    // Compliment of the inner intervals
    comp.emplace_back(-std::numeric_limits<double>::max(), innerCopy.at(0).s());
    for (size_t i = 0; i < innerCopy.size() - 1; ++i) {
        comp.emplace_back(innerCopy.at(i).e(), innerCopy.at(i + 1).s());
    }
    comp.emplace_back(innerCopy.back().e(), std::numeric_limits<double>::max());

    // Intersection with outer interval
    for (auto curr : comp) {
        intBuff.push_back(curr);
        intBuff.push_back(outer.at(0));

        intsect = intersects(intBuff);
        if (!intsect.empty()) {
            res.push_back(intsect.at(0));
        }

        intBuff.clear();
    }

    return res;
}
