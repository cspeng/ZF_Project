#include "common.h"

#include <assert.h>
#include <ctype.h>

#if HAVE_SYS_SOCKET_H
#include <sys/socket.h>
#endif /* HAVE_SYS_SOCKET_H */

#if HAVE_SYS_STAT_H
#include <sys/stat.h>
#endif /* HAVE_SYS_STAT_H */

#if HAVE_NETINET_IN_H
#include <netinet/in.h>
#endif /* HAVE_NETINET_IN_H */

#if defined(UNIXODBC) || defined(_WIN32)
#include <odbcinst.h>
#endif

#ifndef _WIN32
#include <freetds/sysdep_private.h>
#else
#define TDS_SDIR_SEPARATOR "\\"
#endif

HENV odbc_env;
HDBC odbc_conn;
HSTMT odbc_stmt;
int odbc_use_version3 = 0;
void (*odbc_set_conn_attr)(void) = NULL;

char odbc_user[512];
char odbc_server[512];
char odbc_password[512];
char odbc_database[512];
char odbc_driver[1024];

static int
check_lib(char *path, const char *file)
{
	int len = strlen(path);
	FILE *f;

	strcat(path, file);
	f = fopen(path, "rb");
	if (f) {
		fclose(f);
		return 1;
	}
	path[len] = 0;
	return 0;
}

/* some platforms do not have setenv, define a replacement */
#if !HAVE_SETENV
void
odbc_setenv(const char *name, const char *value, int overwrite)
{
#if HAVE_PUTENV
	char buf[1024];

	sprintf(buf, "%s=%s", name, value);
	putenv(buf);
#endif
}
#endif

/* this should be extended with all possible systems... */
static const char *const search_driver[] = {
	".libs/libtdsodbc.so",
	".libs/libtdsodbc.sl",
	".libs/libtdsodbc.dylib",
	".libs/libtdsodbc.dll",
	"_libs/libtdsodbc.dll",
	"debug/tdsodbc.dll",
	"release/tdsodbc.dll",
	"libtdsodbc.so",
	"tdsodbc.dll",
	NULL
};

int
odbc_read_login_info(void)
{
	static const char *PWD = "../../../PWD";
	FILE *in = NULL;
	char line[512];
	char *s1, *s2;
	const char *const *search_p;
	char path[1024];
	int len;
#ifdef _WIN32
	UWORD old_config_mode;
#endif

	setbuf(stdout, NULL);
	setbuf(stderr, NULL);

	s1 = getenv("TDSPWDFILE");
	if (s1 && s1[0])
		in = fopen(s1, "r");
	if (!in)
		in = fopen(PWD, "r");
	if (!in)
		in = fopen("PWD", "r");

	if (!in) {
		fprintf(stderr, "Can not open PWD file\n\n");
		return 1;
	}
	while (fgets(line, 512, in)) {
		s1 = strtok(line, "=");
		s2 = strtok(NULL, "\n");
		if (!s1 || !s2)
			continue;
		if (!strcmp(s1, "UID")) {
			strcpy(odbc_user, s2);
		} else if (!strcmp(s1, "SRV")) {
			strcpy(odbc_server, s2);
		} else if (!strcmp(s1, "PWD")) {
			strcpy(odbc_password, s2);
		} else if (!strcmp(s1, "DB")) {
			strcpy(odbc_database, s2);
		}
	}
	fclose(in);

	/* find our driver */
#ifndef _WIN32
	if (!getcwd(path, sizeof(path)))
#else
	if (!_getcwd(path, sizeof(path)))
#endif
		return 0;
#ifdef __VMS
	{
	    /* A hard-coded driver path has to be in unix syntax to be recognized as such. */
	    const char *unixspec = decc$translate_vms(path);
	    if ( (int)unixspec != 0 && (int)unixspec != -1 ) strcpy(path, unixspec);
	}
#endif
	len = strlen(path);
	if (len < 10 || (strcasecmp(path + len - 10, "/unittests") != 0
#ifdef _WIN32
	    && strcasecmp(path + len - 10, "\\unittests") != 0
#endif
	    ))
		return 0;
	path[len - 9] = 0;
	for (search_p = search_driver; *search_p; ++search_p) {
		if (check_lib(path, *search_p))
			break;
	}
	if (!*search_p)
		return 0;
	strcpy(odbc_driver, path);

#ifndef _WIN32
	/* craft out odbc.ini, avoid to read wrong one */
	sprintf(path, "odbc.ini.%d", (int) getpid());
	in = fopen(path, "w");
	if (in) {
		fprintf(in, "[%s]\nDriver = %s\nDatabase = %s\nServername = %s\n", odbc_server, odbc_driver, odbc_database, odbc_server);
		fclose(in);
		setenv("ODBCINI", "./odbc.ini", 1);
		setenv("SYSODBCINI", "./odbc.ini", 1);
		rename(path, "odbc.ini");
		unlink(path);
	}
#else
	if (SQLGetConfigMode(&old_config_mode)) {
		SQLSetConfigMode(ODBC_USER_DSN);
		SQLWritePrivateProfileString(odbc_server, "Driver", odbc_driver, "odbc.ini");
		SQLWritePrivateProfileString(odbc_server, "Database", odbc_database, "odbc.ini");
		SQLWritePrivateProfileString(odbc_server, "Servername", odbc_server, "odbc.ini");
		SQLSetConfigMode(old_config_mode);
	}
#endif
	return 0;
}

void
odbc_report_error(const char *errmsg, int line, const char *file)
{
	SQLSMALLINT handletype;
	SQLHANDLE handle;
	SQLRETURN ret;
	SQLTCHAR sqlstate[6];
	SQLTCHAR msg[256];
	ODBC_BUF *odbc_buf = NULL;

	if (odbc_stmt) {
		handletype = SQL_HANDLE_STMT;
		handle = odbc_stmt;
	} else if (odbc_conn) {
		handletype = SQL_HANDLE_DBC;
		handle = odbc_conn;
	} else {
		handletype = SQL_HANDLE_ENV;
		handle = odbc_env;
	}
	if (errmsg[0]) {
		if (line)
			fprintf(stderr, "%s:%d %s\n", file, line, errmsg);
		else
			fprintf(stderr, "%s\n", errmsg);
	}
	ret = SQLGetDiagRec(handletype, handle, 1, sqlstate, NULL, msg, ODBC_VECTOR_SIZE(msg), NULL);
	if (ret == SQL_SUCCESS || ret == SQL_SUCCESS_WITH_INFO)
		fprintf(stderr, "SQL error %s -- %s\n", C(sqlstate), C(msg));
	odbc_disconnect();
	ODBC_FREE();
	exit(1);
}

static void
ReportODBCError(const char *errmsg, SQLSMALLINT handletype, SQLHANDLE handle, SQLRETURN rc, int line, const char *file)
{
	SQLRETURN ret;
	SQLTCHAR sqlstate[6];
	SQLTCHAR msg[256];
	ODBC_BUF *odbc_buf = NULL;

	if (errmsg[0]) {
		if (line)
			fprintf(stderr, "%s:%d rc=%d %s\n", file, line, (int) rc, errmsg);
		else
			fprintf(stderr, "rc=%d %s\n", (int) rc, errmsg);
	}
	ret = SQLGetDiagRec(handletype, handle, 1, sqlstate, NULL, msg, ODBC_VECTOR_SIZE(msg), NULL);
	if (ret == SQL_SUCCESS || ret == SQL_SUCCESS_WITH_INFO)
		fprintf(stderr, "SQL error %s -- %s\n", C(sqlstate), C(msg));
	odbc_disconnect();
	ODBC_FREE();
	exit(1);
}

int
odbc_connect(void)
{
	ODBC_BUF *odbc_buf = NULL;
	char command[512];
	const char *p;

	if (odbc_read_login_info())
		exit(1);

	if (odbc_use_version3) {
		CHKAllocHandle(SQL_HANDLE_ENV, SQL_NULL_HANDLE, &odbc_env, "S");
		SQLSetEnvAttr(odbc_env, SQL_ATTR_ODBC_VERSION, (SQLPOINTER) (SQL_OV_ODBC3), SQL_IS_UINTEGER);
		CHKAllocHandle(SQL_HANDLE_DBC, odbc_env, &odbc_conn, "S");
	} else {
		CHKAllocEnv(&odbc_env, "S");
		CHKAllocConnect(&odbc_conn, "S");
	}

	printf("odbctest\n--------\n\n");
	printf("connection parameters:\nserver:   '%s'\nuser:     '%s'\npassword: '%s'\ndatabase: '%s'\n",
	       odbc_server, odbc_user, "????" /* odbc_password */ , odbc_database);

	p = getenv("ODBC_MARS");
	if (p && atoi(p) != 0)
		SQLSetConnectAttr(odbc_conn, 1224 /*SQL_COPT_SS_MARS_ENABLED*/, (SQLPOINTER) 1 /*SQL_MARS_ENABLED_YES*/, SQL_IS_UINTEGER);
	if (odbc_set_conn_attr)
		(*odbc_set_conn_attr)();

	CHKConnect(T(odbc_server), SQL_NTS, T(odbc_user), SQL_NTS, T(odbc_password), SQL_NTS, "SI");

	CHKAllocStmt(&odbc_stmt, "S");

	sprintf(command, "use %s", odbc_database);
	printf("%s\n", command);

	CHKExecDirect(T(command), SQL_NTS, "SI");

#ifndef TDS_NO_DM
	/* unixODBC seems to require it */
	SQLMoreResults(odbc_stmt);
#endif
	ODBC_FREE();
	return 0;
}

int
odbc_disconnect(void)
{
	if (odbc_stmt) {
		SQLFreeStmt(odbc_stmt, SQL_DROP);
		odbc_stmt = SQL_NULL_HSTMT;
	}

	if (odbc_conn) {
		SQLDisconnect(odbc_conn);
		SQLFreeConnect(odbc_conn);
		odbc_conn = SQL_NULL_HDBC;
	}

	if (odbc_env) {
		SQLFreeEnv(odbc_env);
		odbc_env = SQL_NULL_HENV;
	}
	ODBC_FREE();
	return 0;
}

SQLRETURN
odbc_command_with_result(HSTMT stmt, const char *command)
{
	SQLRETURN ret;
	ODBC_BUF *odbc_buf = NULL;

	printf("%s\n", command);
	ret = SQLExecDirect(stmt, T(command), SQL_NTS);
	ODBC_FREE();
	return ret;
}

static int ms_db = -1;
int
odbc_db_is_microsoft(void)
{
	ODBC_BUF *odbc_buf = NULL;
	SQLTCHAR buf[64];
	SQLSMALLINT len;
	int i;

	if (ms_db < 0) {
		buf[0] = 0;
		SQLGetInfo(odbc_conn, SQL_DBMS_NAME, buf, sizeof(buf), &len);
		for (i = 0; buf[i]; ++i)
			buf[i] = tolower(buf[i]);
		ms_db = (strstr(C(buf), "microsoft") != NULL);
	}
	ODBC_FREE();
	return ms_db;
}

static int freetds_driver = -1;
int
odbc_driver_is_freetds(void)
{
	ODBC_BUF *odbc_buf = NULL;
	SQLTCHAR buf[64];
	SQLSMALLINT len;
	int i;

	if (freetds_driver < 0) {
		buf[0] = 0;
		SQLGetInfo(odbc_conn, SQL_DRIVER_NAME, buf, sizeof(buf), &len);
		for (i = 0; buf[i]; ++i)
			buf[i] = tolower(buf[i]);
		freetds_driver = (strstr(C(buf), "tds") != NULL);
	}
	ODBC_FREE();
	return freetds_driver;
}

int
odbc_tds_version(void)
{
	static int tds_version = -1;

	ODBC_BUF *odbc_buf = NULL;
	SQLUINTEGER version;
	SQLSMALLINT len;

	if (!odbc_driver_is_freetds())
		return 0;

	if (tds_version < 0) {
		version = 0;
		len = 0;
		SQLGetInfo(odbc_conn, 1300 /* SQL_INFO_FREETDS_TDS_VERSION */, &version, sizeof(version), &len);
		if (len == sizeof(version))
			tds_version = (version >> 16) << 8 | (version & 0xff);
	}
	ODBC_FREE();
	return tds_version < 0 ? 0: tds_version;
}

static char db_str_version[32];

const char *odbc_db_version(void)
{
	if (!db_str_version[0]) {
		ODBC_BUF *odbc_buf = NULL;
		SQLTCHAR buf[32];
		SQLSMALLINT version_len;

		CHKR(SQLGetInfo, (odbc_conn, SQL_DBMS_VER, buf, sizeof(buf), &version_len), "S");
		strcpy(db_str_version, C(buf));
		ODBC_FREE();
	}

	return db_str_version;
}

unsigned int odbc_db_version_int(void)
{
	unsigned int h, l;
	if (sscanf(odbc_db_version(), "%u.%u.", &h, &l) != 2) {
		fprintf(stderr, "Wrong db version: %s\n", odbc_db_version());
		odbc_disconnect();
		exit(1);
	}

	return (h << 24) | ((l & 0xFFu) << 16);
}

void
odbc_check_cols(int n, int line, const char * file)
{
	SQLSMALLINT cols;

	if (n < 0) {
		CHKNumResultCols(&cols, "E");
		return;
	}
	CHKNumResultCols(&cols, "S");
	if (cols != n) {
		fprintf(stderr, "%s:%d: Expected %d columns returned %d\n", file, line, n, (int) cols);
		odbc_disconnect();
		exit(1);
	}
}

void
odbc_check_rows(int n, int line, const char * file)
{
	SQLLEN rows;

	if (n < -1) {
		CHKRowCount(&rows, "E");
		return;
	}

	CHKRowCount(&rows, "S");
	if (rows != n) {
		fprintf(stderr, "%s:%d: Expected %d rows returned %d\n", file, line, n, (int) rows);
		odbc_disconnect();
		exit(1);
	}
}

void
odbc_reset_statement_proc(SQLHSTMT *stmt, const char *file, int line)
{
	SQLFreeStmt(*stmt, SQL_DROP);
	*stmt = SQL_NULL_HSTMT;
	odbc_check_res(file, line, SQLAllocStmt(odbc_conn, stmt), SQL_HANDLE_DBC, odbc_conn, "SQLAllocStmt", "S");
}

void
odbc_test_skipped(void)
{
	const char *p = getenv("TDS_SKIP_SUCCESS");
	if (p && atoi(p) != 0)
		exit(0);
	exit(77);
}

void
odbc_check_cursor(void)
{
	SQLRETURN retcode;
	ODBC_BUF *odbc_buf = NULL;

	retcode = SQLSetStmtAttr(odbc_stmt, SQL_ATTR_CONCURRENCY, (SQLPOINTER) SQL_CONCUR_ROWVER, 0);
	if (retcode != SQL_SUCCESS) {
		SQLTCHAR output[256];
		SQLTCHAR sqlstate[6];

		CHKGetDiagRec(SQL_HANDLE_STMT, odbc_stmt, 1, sqlstate, NULL, output, ODBC_VECTOR_SIZE(output), NULL, "S");
		sqlstate[5] = 0;
		if (strcmp(C(sqlstate), "01S02") == 0) {
			printf("Your connection seems to not support cursors, probably you are using wrong protocol version or Sybase\n");
			odbc_disconnect();
			ODBC_FREE();
			odbc_test_skipped();
		}
		ReportODBCError("SQLSetStmtAttr", SQL_HANDLE_STMT, odbc_stmt, retcode, __LINE__, __FILE__);
	}
	odbc_reset_statement();
	ODBC_FREE();
}

SQLRETURN
odbc_check_res(const char *file, int line, SQLRETURN rc, SQLSMALLINT handle_type, SQLHANDLE handle, const char *func, const char *res)
{
	const char *p = res;
	for (;;) {
		if (*p == 'S') {
			if (rc == SQL_SUCCESS)
				return rc;
			++p;
		} else if (*p == 'I') {
			if (rc == SQL_SUCCESS_WITH_INFO)
				return rc;
			++p;
		} else if (*p == 'E') {
			if (rc == SQL_ERROR)
				return rc;
			++p;
		} else if (strncmp(p, "No", 2) == 0) {
			if (rc == SQL_NO_DATA)
				return rc;
			p += 2;
		} else if (strncmp(p, "Ne", 2) == 0) {
			if (rc == SQL_NEED_DATA)
				return rc;
			p += 2;
		} else if (!*p) {
			break;
		} else {
			odbc_report_error("Wrong results specified", line, file);
			return rc;
		}
	}
	ReportODBCError(func, handle_type, handle, rc, line, file);
	return rc;
}

SQLSMALLINT
odbc_alloc_handle_err_type(SQLSMALLINT type)
{
	switch (type) {
	case SQL_HANDLE_DESC:
		return SQL_HANDLE_STMT;
	case SQL_HANDLE_STMT:
		return SQL_HANDLE_DBC;
	case SQL_HANDLE_DBC:
		return SQL_HANDLE_ENV;
	}
	return 0;
}

SQLRETURN
odbc_command_proc(HSTMT stmt, const char *command, const char *file, int line, const char *res)
{
	SQLRETURN ret;
	ODBC_BUF *odbc_buf = NULL;

	printf("%s\n", command);
	ret = odbc_check_res(file, line, SQLExecDirect(stmt, T(command), SQL_NTS), SQL_HANDLE_STMT, stmt, "odbc_command", res);
	ODBC_FREE();
	return ret;
}

char odbc_err[512];
char odbc_sqlstate[6];

void
odbc_read_error(void)
{
	ODBC_BUF *odbc_buf = NULL;
	SQLTCHAR *err = ODBC_GET(sizeof(odbc_err)*sizeof(SQLTCHAR));
	SQLTCHAR *state = ODBC_GET(sizeof(odbc_sqlstate)*sizeof(SQLTCHAR));

	memset(odbc_err, 0, sizeof(odbc_err));
	memset(odbc_sqlstate, 0, sizeof(odbc_sqlstate));
	CHKGetDiagRec(SQL_HANDLE_STMT, odbc_stmt, 1, state, NULL, err, sizeof(odbc_err), NULL, "SI");
	strcpy(odbc_err, C(err));
	strcpy(odbc_sqlstate, C(state));
	ODBC_FREE();
	printf("Message: '%s' %s\n", odbc_sqlstate, odbc_err);
}

int
odbc_to_sqlwchar(SQLWCHAR *dst, const char *src, int n)
{
	int i = n;
	while (--i >= 0)
		dst[i] = (unsigned char) src[i];
	return n * sizeof(SQLWCHAR);
}

int
odbc_from_sqlwchar(char *dst, const SQLWCHAR *src, int n)
{
	int i;
	if (n < 0) {
		const SQLWCHAR *p = src;
		for (n=1; *p++ != 0; ++n)
			continue;
	}
	for (i = 0; i < n; ++i) {
		assert(src[i] < 256);
		dst[i] = (char) src[i];
	}
	return n;
}

ODBC_BUF *odbc_buf = NULL;

void *
odbc_buf_get(ODBC_BUF** buf, size_t s)
{
	ODBC_BUF *p = (ODBC_BUF*) calloc(1, sizeof(ODBC_BUF));
	assert(p);
	p->buf = malloc(s);
	assert(p->buf);
	p->next = *buf;
	*buf = p;
	return p->buf;
}

void
odbc_buf_free(ODBC_BUF** buf)
{
	ODBC_BUF *cur = *buf;
	*buf = NULL;
	while (cur) {
		ODBC_BUF *next = cur->next;
		free(cur->buf);
		free(cur);
		cur = next;
	}
}

SQLWCHAR *
odbc_get_sqlwchar(ODBC_BUF** buf, const char *s)
{
	size_t l;
	SQLWCHAR *buffer;

	if (!s) return NULL;
	l = strlen(s) + 1;
	buffer = (SQLWCHAR*) odbc_buf_get(buf, l * sizeof(SQLWCHAR));
	odbc_to_sqlwchar(buffer, s, l);
	return buffer;
}

char*
odbc_get_sqlchar(ODBC_BUF** buf, SQLWCHAR *s)
{
	int n;
	const SQLWCHAR *p = s;
	char *out;

	for (n=1; *p++ != 0; ++n)
		continue;
	out = odbc_buf_get(buf, n);
	odbc_from_sqlwchar(out, s, n);
	return out;
}

typedef union {
	struct sockaddr sa;
	struct sockaddr_in sin;
	char dummy[256];
} long_sockaddr;

static int
fd_is_socket(int fd)
{
	long_sockaddr addr;
	socklen_t addr_len;

#ifndef _WIN32
	struct stat file_stat;

	if (fstat(fd, &file_stat))
		return 0;
	if ((file_stat.st_mode & S_IFSOCK) != S_IFSOCK)
		return 0;
#endif

	addr_len = sizeof(addr);
	if (tds_getpeername((TDS_SYS_SOCKET) fd, &addr.sa, &addr_len))
		return 0;

	addr_len = sizeof(addr);
	if (tds_getsockname((TDS_SYS_SOCKET) fd, &addr.sa, &addr_len))
		return 0;

	return 1;
}

static int
mark_fd(int fd)
{
	enum {NUM_FDS = 4096*4};
	static unsigned char fd_bitmask[NUM_FDS / 8];

	unsigned shift;
	unsigned char mask;

	if (fd < 0 || fd >= NUM_FDS)
		return 0;

	shift = fd & 7;
	mask = fd_bitmask[fd >> 3];
	fd_bitmask[fd >> 3] = mask | (1 << shift);

	return (mask >> shift) & 1;
}

#ifdef _WIN32
#define FOR_ALL_SOCKETS(i) for (i = 4; i <= (4096*4); i += 4)
#else
#define FOR_ALL_SOCKETS(i) for (i = 3; i < 1024; ++i)
#endif

void
odbc_mark_sockets_opened(void)
{
	int i;
	FOR_ALL_SOCKETS(i) {
		if (fd_is_socket(i))
			mark_fd(i);
	}
}

TDS_SYS_SOCKET
odbc_find_last_socket(void)
{
	typedef struct {
		TDS_SYS_SOCKET sock;
		int local_port;
		int remote_port;
	} sock_info;
	sock_info found[8];
	unsigned num_found = 0, n;
	int i;

	FOR_ALL_SOCKETS(i) {
		long_sockaddr remote_addr, local_addr;
		struct sockaddr_in *in;
		socklen_t remote_addr_len, local_addr_len;
		sock_info *info;

		/* check if is a socket */
		if (!fd_is_socket(i))
			continue;
		if (mark_fd(i))
			continue;

		remote_addr_len = sizeof(remote_addr);
		if (tds_getpeername((TDS_SYS_SOCKET) i, &remote_addr.sa, &remote_addr_len))
			continue;
		if (remote_addr.sa.sa_family != AF_INET
#ifdef AF_INET6
		    && remote_addr.sa.sa_family != AF_INET6
#endif
		    )
			continue;
		local_addr_len = sizeof(local_addr);
		if (tds_getsockname((TDS_SYS_SOCKET) i, &local_addr.sa, &local_addr_len))
			continue;

		/* save in the array */
		if (num_found >= 8) {
			memmove(found, found+1, sizeof(found) - sizeof(found[0]));
			num_found = 7;
		}
		info = &found[num_found++];
		info->sock = (TDS_SYS_SOCKET) i;
		info->local_port = -1;
		info->remote_port = -1;

		/* now check if is a socketpair */
		in = &remote_addr.sin;
		if (in->sin_family != AF_INET)
			continue;
		if (in->sin_addr.s_addr != htonl(INADDR_LOOPBACK))
			continue;
		info->remote_port = ntohs(in->sin_port);
		in = &local_addr.sin;
		if (in->sin_family != AF_INET)
			continue;
		if (in->sin_addr.s_addr != htonl(INADDR_LOOPBACK))
			continue;
		info->local_port = ntohs(in->sin_port);
		for (n = 0; n < num_found - 1; ++n) {
			if (found[n].remote_port != info->local_port
			    || found[n].local_port != info->remote_port)
				continue;
			--num_found;
			memmove(found+n, found+n+1, num_found-n-1);
			--num_found;
			break;
		}
	}

	/* return last */
	if (num_found == 0)
		return INVALID_SOCKET;
	return found[num_found-1].sock;
}
