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
    
            typedef typename std::map<K,V>::iterator iterator;
    
        private:
    
    	    map_t map_;
    	    deque_t deque_;
            int size;
    
    	    void _ensure(); 
    
        public:
    
            circularMap();
            circularMap(int size);
    	    iterator begin();
            iterator end();
            iterator find_upper(K key);
            iterator find_lower(K key);
            iterator find(K key);
            bool empty();
    	    void put(K k, V v);
    };
}

#endif
