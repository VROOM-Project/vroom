#ifndef PREPOSTVIEW_H
#define PREPOSTVIEW_H

#include <iterator>
#include <vector>


template<typename T>
class prepostview
{
    public:
        prepostview(T const & value, std::vector<T> const & vector)
          : prepostview(value, vector, 0, vector.size())
        {
        }

        prepostview(std::vector<T> const & vector, T const & value)
          : prepostview(vector, value, 0)
        {
        }

        prepostview(T const & value, std::vector<T> const & vector, std::size_t start)
          : prepostview(value, vector, start, vector.size())
        {
        }

        prepostview(std::vector<T> const & vector, T const & value, std::size_t start)
          : prepostview(vector, value, start, vector.size())
        {
        }

        prepostview(T const & value, std::vector<T> const & vector, std::size_t start, std::size_t end)
          : m_vector(&vector)
          , m_value(value)
          , m_prepended(true)
          , m_start(start)
          , m_end(end)
        {
        }

        prepostview(std::vector<T> const & vector, T const & value, std::size_t start, std::size_t end)
          : m_vector(&vector)
          , m_value(value)
          , m_prepended(false)
          , m_start(start)
          , m_end(end)
        {
        }

        prepostview()
          : m_vector(nullptr)
          , m_value()
          , m_prepended(false)
          , m_start(0)
          , m_end(0)
        {
        }

        auto operator[](std::size_t index) const -> const T &
        {
            if (index == m_start - 1 && m_prepended)
            {
                return m_value;
            }

            if (index == m_end && !m_prepended)
            {
                return m_value;
            }

            return (*m_vector)[index];
        }

        class iterator
        {
            public:
                iterator(prepostview const & ppv, std::size_t index)
                  : m_ppv(&ppv)
                  , m_index(index)
                {
                }

                iterator(iterator const & other)
                  : m_ppv(other.m_ppv)
                  , m_index(other.m_index)
                {
                }

                iterator()
                  : m_ppv(nullptr)
                  , m_index(-2)
                {
                }

                auto operator==(iterator const & other) const -> bool
                {
                    return m_index == other.m_index;
                }

                auto operator!=(iterator const & other) const -> bool
                {
                    return m_index != other.m_index;
                }

                auto operator<(iterator const & other) const -> bool
                {
                    if (m_index == other.m_index)
                    {
                        return false;
                    }

                    if (m_index == -1)
                    {
                        return true;
                    }

                    return m_index < other.m_index;
                }

                auto operator<=(iterator const & other) const -> bool
                {
                    return !(other < *this);
                }

                auto operator*() const -> T
                {
                    return (*m_ppv)[m_index];
                }

                auto operator->() -> T &
                {
                    return &((*m_ppv)[m_index]);
                }

                auto operator++() -> iterator &
                {
                    ++m_index;
                    return *this;
                }

                auto operator++(int) -> iterator
                {
                    auto tmp = *this;
                    ++(*this);
                    return tmp;
                }

                auto operator-(int i) const -> iterator
                {
                    return iterator(*m_ppv, m_index - i);
                }

                // using iterator_category = std::forward_iterator_tag;
                using difference_type = std::ptrdiff_t;
                using element_type = T;
                // using pointer = T *;
                // using reference = T &;

            private:
                prepostview const * m_ppv;
                std::size_t m_index;
        };

        auto begin() const -> iterator
        {
            return iterator(*this, m_prepended ? m_start - 1 : m_start);
        }

        auto end() const -> iterator
        {
            return iterator(*this, m_prepended ? m_end : m_end + 1);
        }

    private:
        std::vector<T> const * m_vector;
        T m_value;
        bool m_prepended;
        std::size_t m_start;
        std::size_t m_end;
};

#endif
