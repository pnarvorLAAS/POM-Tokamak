//This was inspired by the example given in https://stackoverflow.com/questions/7153532/c-how-to-mix-a-map-with-a-circular-buffer

template <typename K, typename V>
struct circularMap
{
	typedef std::map<K,V> map_t;
	typedef std::deque<K> deque_t;

	// ...
	typedef value_type map_t::value_type;
	// Reuse map's iterators
	typedef iterator map_t::iterator;

	// ...
	iterator begin() { return map_.begin(); }

	// put
	void put ( K k, V v)
	{ 
		map_.insert(std::make_pair(k,v));
		deque_.push_back(k);
		_ensure();  // ensure the size of the map, and remove the last element
	}

	// ...

	private:

	map_t map;
	deque_t deque;

	void _ensure() 
	{ 
		if (deque_size() > LIMIT) 
		{ 
			map.erase(deque_.front()); 
			deque.pop_front();
		}
	}

}
