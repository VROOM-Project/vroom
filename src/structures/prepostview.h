#ifndef PREPOSTVIEW_H
#define PREPOSTVIEW_H

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
                  : m_ppv(ppv)
                  , m_index(index)
                {
                }

                auto operator!=(iterator const & other) const -> bool
                {
                    return m_index != other.m_index;
                }

                auto operator*() const -> T
                {
                    return m_ppv[m_index];
                }

                auto operator++() -> iterator &
                {
                    ++m_index;
                    return *this;
                }

                using iterator_category = std::forward_iterator_tag;

            private:
                prepostview const & m_ppv;
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
