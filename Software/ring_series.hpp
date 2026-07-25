//===================================================================================================//
//                                          Ring series buffer                                       //
//===================================================================================================//
// Fixed-length rolling buffer, equivalent to Python's collections.deque(maxlen=N).
// Contiguous storage + write cursor so plotting can hand ImPlot a pointer directly
// (ImPlot::PlotLine supports a circular "offset" natively -> zero-copy per frame).
#pragma once
#include <vector>
#include <cstddef>

class RingSeries {
public:
    explicit RingSeries(int n) : data_(n, 0.0f), size_(n) {}

    void push(float v) {
        data_[cursor_] = v;
        cursor_ = (cursor_ + 1) % size_;
    }

    const float* data() const { return data_.data(); }
    int size()   const { return size_; }
    int offset() const { return cursor_; }

private:
    std::vector<float> data_;
    int size_ = 0;
    int cursor_ = 0;
};
