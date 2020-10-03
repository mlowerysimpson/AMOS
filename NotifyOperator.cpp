//NotifyOperator.cpp
//Implementation file for NotifyOperator class

#include "NotifyOperator.h"
#include "ShipLog.h"
#include "filedata.h"
#include <curl/curl.h>
#include <ctime>

char *g_payload_text[MAX_EMAIL_ITEMS];//the text that gets sent in the email

NotifyOperator::NotifyOperator() {
	m_bSendText=false;
	m_twilio_account_sid=nullptr;
	m_twilio_auth_token=nullptr;
	m_fromNumber=nullptr;
	m_bSendEmail=false;
	m_fromAddr=nullptr;//where the email is from
	m_emailUsername=nullptr;//username for the email account that is sending the email
	m_emailPassword=nullptr;//password for the email account that is sending the email
	m_nMailServerPortNumber=0;//the port number used by the mailserver
	m_mailServerName=nullptr;//the name of the mailserver
	m_bLoadedConfigInfo = LoadConfigInfo();//load configuration data from prefs.txt file, return true if file could be successfully opened and read
}

NotifyOperator::~NotifyOperator() {
	if (m_twilio_account_sid) {
		delete []m_twilio_account_sid;
	}
	if (m_twilio_auth_token) {
		delete []m_twilio_auth_token;
	}
	int nNumToNumbers = m_toNumbers.size();
	for (int i=0;i<nNumToNumbers;i++) {
		if (m_toNumbers[i]) {
			delete []m_toNumbers[i];
			m_toNumbers[i]=nullptr;
		}
	}
	m_toNumbers.clear();
	if (m_fromNumber) {
		delete []m_fromNumber;
		m_fromNumber=nullptr;
	}
	if (m_fromAddr) {
		delete []m_fromAddr;
		m_fromAddr=nullptr;
	}
	int nNumEmailAddresses = m_emailAddressees.size();
	for (int i=0;i<nNumEmailAddresses;i++) {
		if (m_emailAddressees[i]) {
			delete []m_emailAddressees[i];
			m_emailAddressees[i]=nullptr;
		}
	}
	m_emailAddressees.clear();
	if (m_emailUsername) {
		delete []m_emailUsername;
		m_emailUsername=nullptr;
	}
	if (m_emailPassword) {
		delete []m_emailPassword;
		m_emailPassword=nullptr;
	}
	if (m_mailServerName) {
		delete []m_mailServerName;
		m_mailServerName=nullptr;
	}
}

/**
 * @brief send out text(s) and email(s) to configured recipients
 * 
 * @param msgText the notification text to send
 * @param szSubject the subject text to include in the email message (if used)
 * @param pShipLog void pointer to the ship's log file
 * @return true if notification was issued successfully
 * @return false if an error occurred
 */
bool NotifyOperator::IssueNotification(char *msgText, char *szSubject, void *pShipLog) {
	ShipLog *pLog = (ShipLog *)pShipLog;
	if (!m_bLoadedConfigInfo) {
		pLog->LogEntry((char *)"Error, unable to load email / text configuration info from prefs.txt file.\n",true);
		return false;
	}
	bool bRetval = true;
	if (m_bSendText) {
		if (!SendText(msgText,pLog)) {
			pLog->LogEntry((char *)"Error, failed to send text.\n",true);
			bRetval=false;
		}
	}
	if (m_bSendEmail) {
		if (!SendEmail(msgText,szSubject,pLog)) {
			pLog->LogEntry((char *)"Error, failed to send email.\n",true);
			bRetval=false;
		}
	}
	return bRetval;
}

bool NotifyOperator::SendText(char *msgText,void *pShipLog) {//send text message to one or more recipients
	// Instantiate a twilio object and call send_message
	std::string response;
	char sMsg[256];
	ShipLog *pLog = (ShipLog *)pShipLog;
	if (!m_twilio_account_sid) {
		pLog->LogEntry((char *)"No Twilio account ID available to send text message.\n",true);
		return false;
	}
	if (!m_twilio_auth_token) {
		pLog->LogEntry((char *)"No Twilio authorization token available to send text message.\n",true);
		return false;
	}
	if (!m_fromNumber) {
		pLog->LogEntry((char *)"No \"from\" number specified for sending text message.\n",true);
		return false;
	}
	auto twilio = std::make_shared<twilio::Twilio>(m_twilio_account_sid, 
		m_twilio_auth_token);
	bool bSuccess = true;

	int nNumRecipients = m_toNumbers.size();
	for (int i=0;i<nNumRecipients;i++) {
		if (!twilio->send_message(m_toNumbers[i], m_fromNumber, msgText,
             response, "", false)) {
             bSuccess=false;
			 sprintf(sMsg,"Failed to send text message to: %s\n",m_toNumbers[i]);
		     pLog->LogEntry(sMsg,true);
		}
	}
	pLog->LogEntry((char *)"Sent text message.\n",true);
	return true;
}

bool NotifyOperator::SendEmail(char *msgText,char *szSubjectText, void *pShipLog) {//send email message to one or more recipients
	
	ShipLog *pLog = (ShipLog *)pShipLog;
	char sMsg[256];

	char szDateTime[128];//used to hold the current date and time

	//get date/time stamp corresponding to current date and time
	time_t rawtime;
	struct tm * timeinfo;
	time (&rawtime);
	timeinfo = localtime (&rawtime);

	//format date and time into szDateTime
	strftime(szDateTime, 128, "Date: %a, %d %b %Y %H:%M:%S %z\r\n",timeinfo);
      
	char *email_recipients = GetEmailRecipientList();
	if (!email_recipients) {
		pLog->LogEntry((char *)"No recipients to send email to!\n",true);
		return false;
	}

	char *sender = new char[strlen(m_fromAddr)+20];
	strcpy(sender,"From: ");
	strcat(sender,m_fromAddr);
	strcat(sender,"\r\n");
	
	g_payload_text[0] = szDateTime;
	g_payload_text[1] = email_recipients;//,//"To: " TO "\r\n",
	g_payload_text[2] = sender;//"From: " FROM " (Example User)\r\n",
	g_payload_text[3] = szSubjectText;
	g_payload_text[4] = (char *)"\r\n"; /* empty line to divide headers from body, see RFC5322 */ 
	g_payload_text[5] = msgText;
	g_payload_text[6] =	(char *)"\r\n";
	g_payload_text[7] = (char *)"\r\n";
	
	for (int i=8;i<MAX_EMAIL_ITEMS;i++) {
		g_payload_text[i] = NULL;
	}

	CURL *curl;
	CURLcode res = CURLE_OK;
	struct curl_slist *recipients = NULL;
	struct upload_status upload_ctx;

	upload_ctx.lines_read = 0;

	curl = curl_easy_init();

	if(curl) {
		/* Set username and password */ 
		curl_easy_setopt(curl, CURLOPT_USERNAME, m_emailUsername);
		curl_easy_setopt(curl, CURLOPT_PASSWORD, m_emailPassword);

		/* The URL for your mailserver.*/ 
		char mailServerAddr[256];
		sprintf(mailServerAddr,"%s:%d",m_mailServerName,m_nMailServerPortNumber);
		curl_easy_setopt(curl, CURLOPT_URL, mailServerAddr);


		/* Start with a plain text connection, and upgrade
		* to Transport Layer Security (TLS) using the STARTTLS command. Be careful
		* of using CURLUSESSL_TRY here, because if TLS upgrade fails, the transfer
		* will continue anyway - see the security discussion in the libcurl
		* tutorial for more details. */ 
		curl_easy_setopt(curl, CURLOPT_USE_SSL, (long)CURLUSESSL_ALL);

		/* If your server doesn't have a valid certificate, then you can disable
		* part of the Transport Layer Security protection by setting the
		* CURLOPT_SSL_VERIFYPEER and CURLOPT_SSL_VERIFYHOST options to 0 (false).*/
		curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L);
		curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0L);
		/* That is, in general, a bad idea. It is still better than sending your
		* authentication details in plain text though.  Instead, you should get
		* the issuer certificate (or the host certificate if the certificate is
		* self-signed) and add it to the set of certificates that are known to
		* libcurl using CURLOPT_CAINFO and/or CURLOPT_CAPATH. See docs/SSLCERTS
		* for more information. */ 
		//curl_easy_setopt(curl, CURLOPT_CAINFO, "/path/to/certificate.pem");

		/* Note that this option isn't strictly required, omitting it will result
		* in libcurl sending the MAIL FROM command with empty sender data. All
		* autoresponses should have an empty reverse-path, and should be directed
		* to the address in the reverse-path which triggered them. Otherwise,
		* they could cause an endless loop. See RFC 5321 Section 4.5.5 for more
		* details.
		*/ 

		curl_easy_setopt(curl, CURLOPT_MAIL_FROM, m_fromAddr);

		/* Add two recipients, in this particular case they correspond to the
		* To: and Cc: addressees in the header, but they could be any kind of
		* recipient. */ 
		int nNumRecipients = m_emailAddressees.size();
		for (int i=0;i<nNumRecipients;i++) {
			recipients = curl_slist_append(recipients, m_emailAddressees[i]);
		}
		curl_easy_setopt(curl, CURLOPT_MAIL_RCPT, recipients);
	
		/* We're using a callback function to specify the payload (the headers and
		* body of the message). You could just use the CURLOPT_READDATA option to
		* specify a FILE pointer to read from. */ 
		curl_easy_setopt(curl, CURLOPT_READFUNCTION, payload_source);
		curl_easy_setopt(curl, CURLOPT_READDATA, &upload_ctx);
		curl_easy_setopt(curl, CURLOPT_UPLOAD, 1L);

		/* It is useful to turn on debug 
		* information within libcurl to see what is happening during the transfer.
		*/ 
		curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);

		/* Send the message */ 
		res = curl_easy_perform(curl);

		/* Check for errors */ 
		if(res != CURLE_OK) {
			snprintf(sMsg,256,"curl_easy_perform() failed: %s\n",curl_easy_strerror(res));
			pLog->LogEntry(sMsg,true);
			return false;
		}
		/* Free the list of recipients */ 
		curl_slist_free_all(recipients);
		/* Always cleanup */ 
		curl_easy_cleanup(curl);
	}
	pLog->LogEntry((char *)"Sent email message.\n",true);
	delete []email_recipients;
	delete []sender;
	return true;
}


size_t NotifyOperator::payload_source(void *ptr, size_t size, size_t nmemb, void *userp)
{
  struct upload_status *upload_ctx = (struct upload_status *)userp;
  const char *data;
 
  if((size == 0) || (nmemb == 0) || ((size*nmemb) < 1)) {
    return 0;
  }

  data = g_payload_text[upload_ctx->lines_read];
  
  if(data) {
    size_t len = strlen(data);
    memcpy(ptr, data, len);
    upload_ctx->lines_read++;

    return len;
  }
  return 0;
}

char *NotifyOperator::GetEmailRecipientList() {//gets a character string of all the email recipients. Calling function is responsible for deleting the returned string
	int nNumRecipients = m_emailAddressees.size();
	if (nNumRecipients<=0) {
		return nullptr;
	}
	//estimate total size of buffer required
	int nRequiredBufSize = 10;
	for (int i=0;i<nNumRecipients;i++) {
		nRequiredBufSize+=(strlen(m_emailAddressees[i])+10);//
	}
	char *retVal = new char[nRequiredBufSize];
	strcpy(retVal,"To: ");
	for (int i=0;i<nNumRecipients;i++) {
		strcat(retVal, m_emailAddressees[i]);
		if (i<(nNumRecipients-1)) {
			strcat(retVal,"; ");
		}
	}
	return retVal;
}

bool NotifyOperator::LoadConfigInfo() {//load configuration data from prefs.txt file, return true if file could be successfully opened and read
	if (filedata::getFileLength((char *)"prefs.txt")<=0) {//unable to open file, or file does not have any info in it
		return false;
	}
	filedata prefsFile((char *)"prefs.txt");
	//texting parameters
	m_bSendText = (bool)prefsFile.getInteger((char *)"[text_prefs]",(char *)"send_text");//flag is true if texts should be sent
	m_twilio_account_sid=NULL;//Twilio account ID (NULL if not available or used), used for sending out texts 
	prefsFile.getString((char *)"[text_prefs]",(char *)"twilio_account_sid",&m_twilio_account_sid);
	
	m_twilio_auth_token=NULL;//Twilio authorization token (NULL if not available or used), used for sending out texts
	prefsFile.getString((char *)"[text_prefs]",(char *)"twilio_auth_token",&m_twilio_auth_token);
	
	//list of phone numbers to send text messages to
	char szPhoneNumLabel[32];
	int nCount=0;
	sprintf(szPhoneNumLabel,"phone_num_%03d",(nCount+1));
	while (prefsFile.FindString((char *)"[text_prefs]",szPhoneNumLabel)>=0) {
		char *szPhoneNumber = NULL;
		prefsFile.getString((char *)"[text_prefs]",szPhoneNumLabel,&szPhoneNumber);
		if (szPhoneNumber) {
			m_toNumbers.push_back(szPhoneNumber);
		}
		nCount++;
		sprintf(szPhoneNumLabel,"phone_num_%03d",(nCount+1));
	}
	
	//the Twilio assigned phone # for sending text messages
	m_fromNumber=NULL;
	prefsFile.getString((char *)"[text_prefs]",(char *)"from_number",&m_fromNumber);
	
	//email parameters
	m_bSendEmail=(bool)prefsFile.getInteger((char *)"[email_prefs]",(char *)"send_email");//flag is true if email should be sent
	//where the email is from
	m_fromAddr=NULL;
	prefsFile.getString((char *)"[email_prefs]",(char *)"from_addr",&m_fromAddr);

	//list of email addresses that we are sending messages to
	nCount=0;
	char szEmailLabel[32];
	sprintf(szEmailLabel,"email_addr_%03d",(nCount+1));
	while (prefsFile.FindString((char *)"[email_prefs]",szEmailLabel)>=0) {
		char *szEmailAddr = NULL;
		prefsFile.getString((char *)"[email_prefs]",szEmailLabel,&szEmailAddr);
		if (szEmailAddr) {
			m_emailAddressees.push_back(szEmailAddr);
		}
		nCount++;
		sprintf(szEmailLabel,"email_addr_%03d",(nCount+1));
	}
	
	//username for the email account that is sending the email
	m_emailUsername=NULL;
	prefsFile.getString((char *)"[email_prefs]",(char *)"user_name",&m_emailUsername);
	//password for the email account that is sending the email
	m_emailPassword=NULL;
	prefsFile.getString((char *)"[email_prefs]",(char *)"password",&m_emailPassword);
	//the port number used by the mailserver
	m_nMailServerPortNumber = prefsFile.getInteger((char *)"[email_prefs]",(char *)"port_number");
	//the name of the mailserver
	m_mailServerName=NULL;
	prefsFile.getString((char *)"[email_prefs]",(char *)"server_name",&m_mailServerName);
	return true;
}

bool NotifyOperator::ReloadConfigInfo(char* szRootFolder) {//reload config info from prefs.txt file
	char prefsFilename[PATH_MAX];
	sprintf(prefsFilename, (char*)"%s//prefs.txt", szRootFolder);
	if (filedata::getFileLength(prefsFilename) <= 0) {
		//prefs.txt configuration file does not exist or is empty
		return false;
	}
	filedata prefsFile(prefsFilename);
	//texting parameters
	m_bSendText = (bool)prefsFile.getInteger((char*)"[text_prefs]", (char*)"send_text");//flag is true if texts should be sent
	if (m_twilio_account_sid != NULL) {
		delete[]m_twilio_account_sid;
		m_twilio_account_sid = NULL;
	}
	prefsFile.getString((char*)"[text_prefs]", (char*)"twilio_account_sid", &m_twilio_account_sid);

	if (m_twilio_auth_token!=NULL) {
		delete[]m_twilio_auth_token;
		m_twilio_auth_token = NULL;
	}
	prefsFile.getString((char*)"[text_prefs]", (char*)"twilio_auth_token", &m_twilio_auth_token);

	//list of phone numbers to send text messages to
	int nNumToNumbers = m_toNumbers.size();
	for (int i = 0; i < nNumToNumbers; i++) {
		if (m_toNumbers[i]) {
			delete[]m_toNumbers[i];
			m_toNumbers[i] = nullptr;
		}
	}
	m_toNumbers.clear();
	char szPhoneNumLabel[32];
	int nCount = 0;
	sprintf(szPhoneNumLabel, "phone_num_%03d", (nCount + 1));
	while (prefsFile.FindString((char*)"[text_prefs]", szPhoneNumLabel) >= 0) {
		char* szPhoneNumber = NULL;
		prefsFile.getString((char*)"[text_prefs]", szPhoneNumLabel, &szPhoneNumber);
		if (szPhoneNumber) {
			m_toNumbers.push_back(szPhoneNumber);
		}
		nCount++;
		sprintf(szPhoneNumLabel, "phone_num_%03d", (nCount + 1));
	}

	//the Twilio assigned phone # for sending text messages
	if (m_fromNumber != NULL) {
		delete[] m_fromNumber;
		m_fromNumber = NULL;
	}
	prefsFile.getString((char*)"[text_prefs]", (char*)"from_number", &m_fromNumber);

	//email parameters
	m_bSendEmail = (bool)prefsFile.getInteger((char*)"[email_prefs]", (char*)"send_email");//flag is true if email should be sent
	//where the email is from
	if (m_fromAddr != NULL) {
		delete[] m_fromAddr;
		m_fromAddr = NULL;
	}
	prefsFile.getString((char*)"[email_prefs]", (char*)"from_addr", &m_fromAddr);

	//list of email addresses that we are sending messages to
	int nNumEmailAddressees = m_emailAddressees.size();
	for (int i = 0; i < nNumEmailAddressees; i++) {
		if (m_emailAddressees[i] != NULL) {
			delete[] m_emailAddressees[i];
			m_emailAddressees[i] = NULL;
		}
	}
	nCount = 0;
	char szEmailLabel[32];
	sprintf(szEmailLabel, "email_addr_%03d", (nCount + 1));
	while (prefsFile.FindString((char*)"[email_prefs]", szEmailLabel) >= 0) {
		char* szEmailAddr = NULL;
		prefsFile.getString((char*)"[email_prefs]", szEmailLabel, &szEmailAddr);
		if (szEmailAddr) {
			m_emailAddressees.push_back(szEmailAddr);
		}
		nCount++;
		sprintf(szEmailLabel, "email_addr_%03d", (nCount + 1));
	}

	//username for the email account that is sending the email
	if (m_emailUsername != NULL) {
		delete[] m_emailUsername;
		m_emailUsername = NULL;
	}
	prefsFile.getString((char*)"[email_prefs]", (char*)"user_name", &m_emailUsername);
	//password for the email account that is sending the email
	if (m_emailPassword != NULL) {
		delete[] m_emailPassword;
		m_emailPassword = NULL;
	}
	prefsFile.getString((char*)"[email_prefs]", (char*)"password", &m_emailPassword);
	//the port number used by the mailserver
	m_nMailServerPortNumber = prefsFile.getInteger((char*)"[email_prefs]", (char*)"port_number");
	//the name of the mailserver
	if (m_mailServerName != NULL) {
		delete[] m_mailServerName;
		m_mailServerName = NULL;
	}
	prefsFile.getString((char*)"[email_prefs]", (char*)"server_name", &m_mailServerName);
	return true;
}