#include <my_global.h>
#include <mysql.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int sendValuesToDB(uint16_t pulse, uint16_t power, uint8_t voltage)
{
	char query_str[100], query_cleanup[100];

	//printf("MySQL client version: %s\n", mysql_get_client_info());
	
	MYSQL *con = mysql_init(NULL);
	
	if (con == NULL) 
	{
		//fprintf(stderr, "%s\n", mysql_error(con));
		printf("init connection: %s\n", mysql_error(con));
		return 1;
	}
	
	if (mysql_real_connect(con, "192.168.1.78", "pi", "raspberry", 
			  "logs", 0, NULL, 0) == NULL) 
	{
		//fprintf(stderr, "%s\n", mysql_error(con));
		printf("real connection: %s\n", mysql_error(con));
		mysql_close(con);
		return 1;
	}
	
	sprintf(query_str, "insert into power (pulses, power, time) values (%u, %u, now())", pulse, power);
	
	//printf("q: %s\n", query_str);
	//printf("040\n");
	if (mysql_query(con, query_str)) 
	{
		//fprintf(stderr, "%s\n", mysql_error(con));
		printf("query: %s\n", mysql_error(con));
		mysql_close(con);
		//return 1;
	}
	//printf("055\n");
	//printf("successful q: %s\n", query_str);
	
	// cleanup old values
	sprintf(query_cleanup, "delete from power where time < (now() - interval 1 day)");
	
	//printf("q: %s\n", query_cleanup);
	//printf("040\n");
	if (mysql_query(con, query_cleanup)) 
	{
		//fprintf(stderr, "%s\n", mysql_error(con));
		printf("query cleanup: %s\n", mysql_error(con));
		mysql_close(con);
		//return 1;
	}
	//printf("successful q: %s\n", query_cleanup);
	mysql_close(con);
	
	
	return 0;
}
