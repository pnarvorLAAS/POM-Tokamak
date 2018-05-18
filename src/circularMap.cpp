#include <circularMap.hpp>


circularMap::circularMap()
{
    this->size = DEFAULT_SIZE;
}

circularMap::circularMap(int size)
{
    this->size = size;
}

iterator circularMap::begin() 
{
    return map_.begin(); 
}

iterator circularMap::end() 
{
    return map_.end();
}

iterator circularMap::find_upper(K key)
{
    return map_.upper_bound(key);
}

iterator circularMap::find_lower(K key)
{
    return map_.lower_bound(key);
}

iterator circularMap::find(K key)
{
    return map_.find(key);
}

bool circularMap::empty()
{
    return map_.empty();
}

void circularMap::put(K k, V v)
{ 
	map_.insert(std::make_pair(k,v));
	deque_.push_back(k);
	_ensure();  
}


void circularMap::_ensure() 
{ 
	if (deque_.size() > this->size) 
	{ 
		map_.erase(deque_.front()); 
		deque_.pop_front();
	}
}

