# For developers:

This code styling document contains some general rules for styling to try and keep a coherent look and feel to the code.
Document will probably be updated over time.

## Namescheme
1. Names, except enum values, should use camelCase, where the starting letter is a lower case and every following word starts with a capital letter.
2. Enum values should be full CAPS and an underscore should be used between words.
3. private variables and functions should start with an underscore. struct values that should not be changed by the user should start with a double underscore.

## Code documentation
1. Any function should be documented with doxygen comments.
2. Between the function description and parameter description should be a newline. 3. Aswell as between parameter description and return description.
4. Use proper interpunction and capital letters on doxygen style comments.
5. Use proper [in] and [out] indicators on @param documentation

## Indentation, spacing & brackets
1. Use tabs (4 spaces) for indentation.
2. opening bracket are always on the same line as function names and keywords that use them.
3. if, for and while statements with one line of code should not use any brackets.