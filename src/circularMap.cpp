#include <circularMap.hpp>

namespace tokamak{

    template <typename K, typename V>
    circularMap<K,V>::circularMap()
    {
        this->size = DEFAULT_SIZE;
    }
    
    template <typename K, typename V>
    circularMap<K,V>::circularMap(int size)
    {
        this->size = size;
    }
    
    template <typename K, typename V>
    typename std::map<K,V>::iterator circularMap<K,V>::begin() 
    {
        return map_.begin(); 
    }
    
    template <typename K, typename V>
    typename std::map<K,V>::iterator circularMap<K,V>::end() 
    {
        return map_.end();
    }
    
    template <typename K, typename V>
    typename std::map<K,V>::iterator circularMap<K,V>::find_upper(K key)
    {
        return map_.upper_bound(key);
    }
    
    template <typename K, typename V>
    typename std::map<K,V>::iterator circularMap<K,V>::find_lower(K key)
    {
        return map_.lower_bound(key);
    }
    
    template <typename K, typename V>
    typename std::map<K,V>::iterator circularMap<K,V>::find(K key)
    {
        return map_.find(key);
    }
    
    template <typename K, typename V>
    bool circularMap<K,V>::empty()
    {
        return map_.empty();
    }
    
    template <typename K, typename V>
    void circularMap<K,V>::put(K k, V v)
    { 
    	map_.insert(std::make_pair(k,v));
    	deque_.push_back(k);
    	_ensure();  
    }
    
    
    template <typename K, typename V>
    void circularMap<K,V>::_ensure() 
    { 
    	if (deque_.size() > this->size) 
    	{ 
    		map_.erase(deque_.front()); 
    		deque_.pop_front();
    	}
    }
};

