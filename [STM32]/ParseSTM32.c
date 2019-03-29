
#include <stdio.h>
#include <stdio.h>
#include <string.h>
int main()
{	char str[] = "strtok needs to be called several times to split a string";
	int init_size = strlen(str);
	char delim[] = " ";
    char string[100], sub[100];
    int position, length, c = 0;
	char *ptr = strtok(str, delim);
	while (ptr != NULL)
	{   c=0;
	    
            
        
		printf("'%c'\n", ptr[0]);
		if(ptr[0] == 'G'){}
		if(ptr[0] == 'M'){}
		if(ptr[0] == 'O'){}
		
		
		
		
		
		
		
		ptr = strtok(NULL, delim);
		
	}
    
    return 0;
}
