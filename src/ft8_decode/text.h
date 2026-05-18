#ifndef _INCLUDE_TEXT_H_
#define _INCLUDE_TEXT_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    // Utility functions for characters and strings

    const char* ft8lib_trim_front(const char* str);
    void ft8lib_trim_back(char* str);
    char* ft8lib_trim(char* str);

    char ft8lib_to_upper(char c);
    bool ft8lib_is_digit(char c);
    bool ft8lib_is_letter(char c);
    bool ft8lib_is_space(char c);
    bool ft8lib_in_range(char c, char min, char max);
    bool ft8lib_starts_with(const char* string, const char* prefix);
    bool ft8lib_equals(const char* string1, const char* string2);

    int ft8lib_char_index(const char* string, char c);

    // Text message formatting:
    //   - replaces lowercase letters with uppercase
    //   - merges consecutive spaces into single space
    void ft8lib_fmtmsg(char* msg_out, const char* msg_in);

    // Parse a 2 digit integer from string
    int ft8lib_dd_to_int(const char* str, int length);

    // Convert a 2 digit integer to string
    void ft8lib_int_to_dd(char* str, int value, int width, bool full_sign);

    char ft8lib_charn(int c, int table_idx);
    int ft8lib_nchar(char c, int table_idx);

#ifdef __cplusplus
}
#endif

#endif // _INCLUDE_TEXT_H_
