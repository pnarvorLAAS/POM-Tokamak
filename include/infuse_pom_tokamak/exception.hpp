#ifndef __EXCEPTION_TOKAMAK_HPP__
#define __EXCEPTION_TOKAMAK_HPP__

#include <exception>

using namespace std;

class Error : public exception
{
	public:
		Error(string const& sentence="") throw()
			:m_sentence(sentence)
		{}

		virtual const char* what() const throw()
		{
			return m_sentence.c_str();
		}

		virtual ~Error() throw()
		{}

	private:
		string m_sentence;           
};

#endif
