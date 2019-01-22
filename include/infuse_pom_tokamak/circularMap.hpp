//This was inspired by the example given in https://stackoverflow.com/questions/7153532/c-how-to-mix-a-map-with-a-circular-buffer

#ifndef __CIRCULAR_MAP_HPP__
#define __CIRCULAR_MAP_HPP__

#include <map>
#include <deque>
#include <iostream>

using namespace std;

namespace tokamak
{
#define DEFAULT_SIZE 10000

    template <typename K, typename V>
        class circularMap
        {
            typedef map<K,V> map_t;
            typedef deque<K> deque_t;
            
            public:
            typedef typename std::map<K,V>::iterator iterator;

            private:

            map_t map_;
            deque_t deque_;
            unsigned int size;

            void _ensure()
            {
                if (deque_.size() > this->size) 
                { 
                    map_.erase(deque_.front()); 
                    deque_.pop_front();
                }
            }

            public:

            circularMap()
            {
                this->size = DEFAULT_SIZE;
            }
            circularMap(int size)
            {
                this->size = size;
            }
            iterator begin()
            {
                return map_.begin(); 
            }

            iterator last_element()
            {
                return std::prev(map_.end());
            }

            iterator end()
            {
                return map_.end();
            }
            iterator find_upper(K key)
            {
                return map_.upper_bound(key);
            }
            iterator find_lower(K key)
            {
                return map_.lower_bound(key);
            }
            iterator find(K key)
            {
                return map_.find(key);
            }

            iterator find_closest(K key)
            {
                if (empty())
                {
                    return end();
                }

                iterator it = find_lower(key);
                if (it == end())
                {
                    return std::prev(it);
                }
                else
                {
                    if (key - std::prev(it)->first < it->first - key)
                    {
                        return std::prev(it);
                    }
                    else
                    {
                        return it;
                    }
                }
            }
            bool empty()
            {
                return map_.empty();
            }
            void put(K k, V v)
            { 
                map_.insert(std::make_pair(k,v));
                deque_.push_back(k);
                _ensure();  
            }

            void printSize()
            {
                std::cout <<" Number of elements currently in the timeLine: " <<  deque_.size() << std::endl;
            }

            void print()
            {
                for (iterator it = map_.begin(); it != map_.end(); it++)
                {
                    std::cout << it->first <<", ";
                }
                std::cout << std::endl;
            }

        };
}

#endif
