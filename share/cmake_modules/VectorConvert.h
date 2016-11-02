// -*- C++ -*-
#ifndef VECTORCONVERT_H
#define VECTORCONVERT_H

#include <istream>
#include <ostream>
#include <vector>
#include <string>
#include <coil/stringutil.h>
template < typename T > std::istream & operator>> (std::istream & is,
                                                   std::vector < T > &v)
{
    std::string s;
    std::vector < std::string > sv;
    is >> s;
    sv = coil::split (s, ",");
    v.resize (sv.size ());
    for (int i (0), len (sv.size ()); i < len; ++i)

    {
        T tv;
        if (coil::stringTo (tv, sv[i].c_str ()))

        {
            v[i] = tv;
        }
    }
    return is;
}
#endif // VECTORCONVERT_H
