/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date           			Name      			    Description of Change
 19-8-27                    zhoupeng
============================================================== **/
#include <boost/spirit/include/classic_core.hpp>
#include <boost/spirit/include/phoenix1_primitives.hpp>
#include <boost/spirit/include/phoenix1_operators.hpp>
#include <iostream>
#include <string>

///////////////////////////////////////////////////////////////////////////////
using namespace std;
using namespace BOOST_SPIRIT_CLASSIC_NS;
using namespace phoenix;

///////////////////////////////////////////////////////////////////////////////
//
//  Our adder
//
///////////////////////////////////////////////////////////////////////////////
template <typename IteratorT>
bool adder(IteratorT first, IteratorT last, double& n)
{
    return parse(first, last,

            //  Begin grammar
                 (
                         real_p[var(n) = arg1] >> *(',' >> real_p[var(n) += arg1])
                 )
            ,
            //  End grammar

                 space_p).full;
}

template <typename IteratorT>
bool multiplier(IteratorT first, IteratorT last, double& n)
{
    return parse(first, last,

            //  Begin grammar
                 (
                         real_p[var(n) = arg1] >> *(',' >> real_p[var(n) *= arg1])
                 )
            ,
            //  End grammar

                 space_p).full;
}
////////////////////////////////////////////////////////////////////////////
//
//  Main program
//
////////////////////////////////////////////////////////////////////////////
int
main()
{
    cout << "/////////////////////////////////////////////////////////\n\n";
    cout << "\t\tA parser for summing a list of numbers...\n\n";
    cout << "/////////////////////////////////////////////////////////\n\n";

    cout << "Give me a comma separated list of numbers.\n";
    cout << "The numbers are added using Phoenix.\n";
    cout << "Type [q or Q] to quit\n\n";

    string str;
    while (getline(cin, str))
    {
        if (str.empty() || str[0] == 'q' || str[0] == 'Q')
            break;

        double n;
        if (multiplier(str.begin(), str.end(), n))
        {
            cout << "-------------------------\n";
            cout << "Parsing succeeded\n";
            cout << str << " Parses OK2: " << endl;

            cout << "mul = " << n;
            cout << "\n-------------------------\n";
        }
        else
        {
            cout << "-------------------------\n";
            cout << "Parsing failed\n";
            cout << "-------------------------\n";
        }
    }

    cout << "Bye... :-) \n\n";
    return 0;
}
