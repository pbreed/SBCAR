class FileReporter
{
private:
	FileReporter * m_pNext;
	const char * m_pName;
public:
	FileReporter(const char * name) {m_pNext=pRoot; pRoot=this; m_pName=name; };
static FileReporter * pRoot;
static void DumpList();
static void ShowList();
};

static FileReporter __static_fr( __DATE__ "," __TIME__ "," __BASE_FILE__ );

