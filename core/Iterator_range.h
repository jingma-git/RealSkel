#pragma once

// Class supporting for range loop must implement begin() and end() method
template <typename I>
class Iterator_range : public std::pair<I, I>
{
    typedef std::pair<I, I> Base;

public:
    typedef I iterator;
    typedef I const_iterator;

    Iterator_range(I b, I e) : Base(b, e) {}

    Iterator_range(const std::pair<I, I> &ip) : Base(ip) {}

    I begin() const
    {
        return this->first;
    }

    I end() const
    {
        return this->second;
    }

    std::size_t size() const
    {
        return static_cast<std::size_t>(std::distance(this->first, this->second));
    }

    bool empty() const
    {
        return this->first == this->second;
    }

    // // ToDO: remove this
    // operator std::tuple<I &, I &>()
    // {
    //     std::cout << __FILE__ << " " << __LINE__ << " std::tuple<I &, I &>" << std::endl;
    //     return std::tuple<I &, I &>{this->first, this->second};
    // }

    // operator std::tuple<const I &, const I &>()
    // {
    //     std::cout << __FILE__ << " " << __LINE__ << " std::tuple<const I &, const I &>" << std::endl;
    //     return std::tuple<I &, I &>{this->first, this->second};
    // }
};
