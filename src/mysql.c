#include <my_global.h>
#include <mysql.h>
#include <string.h>

int main(int argc, char **argv)
{
	unsigned short int i;
	char query_str[64];
  printf("MySQL client version: %s\n", mysql_get_client_info());

  MYSQL *con = mysql_init(NULL);

  if (con == NULL) 
  {
      fprintf(stderr, "%s\n", mysql_error(con));
      exit(1);
  }

  if (mysql_real_connect(con, "192.168.1.78", "pi", "raspberry", 
          "logs", 0, NULL, 0) == NULL) 
  {
      fprintf(stderr, "%s\n", mysql_error(con));
      mysql_close(con);
      exit(1);
  }  

  i=0x100;
  sprintf(query_str, "insert into power (power, time) values (%u, now())", i);
  //mysql_select_db('logs');
  if (mysql_query(con, query_str)) 
  {
      fprintf(stderr, "%s\n", mysql_error(con));
      mysql_close(con);
      exit(1);
  }

  mysql_close(con);
  exit(0);
}
