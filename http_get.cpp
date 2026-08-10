//
//    Copyright (C) 2025  Holger Teutsch
//
//    This library is free software; you can redistribute it and/or
//    modify it under the terms of the GNU Lesser General Public
//    License as published by the Free Software Foundation; either
//    version 2.1 of the License, or (at your option) any later version.
//
//    This library is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//    Lesser General Public License for more details.
//
//    You should have received a copy of the GNU Lesser General Public
//    License along with this library; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
//    USA
//

#include <cstdlib>
#include <cstdio>

#include "http_get.h"
#include "log_msg.h"

#if IBM == 1
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <WinHttp.h>

bool
HttpGet(const std::string& url, std::string& data, int timeout)
{
    data.clear();

    DWORD dwSize = 0;
    DWORD dwDownloaded = 0;
    BOOL  bResults = FALSE;
    HINTERNET  hSession = NULL,
               hConnect = NULL,
               hRequest = NULL;

    int result = false;

    int url_len = url.length();
    WCHAR *url_wc = (WCHAR *)alloca((url_len + 1) * sizeof(WCHAR));
    WCHAR *host_wc = (WCHAR *)alloca((url_len + 1) * sizeof(WCHAR));
    WCHAR *path_wc = (WCHAR *)alloca((url_len + 1) * sizeof(WCHAR));

    mbstowcs_s(NULL, url_wc, url_len + 1, url.c_str(), _TRUNCATE);

    URL_COMPONENTS urlComp;
    memset(&urlComp, 0, sizeof(urlComp));
    urlComp.dwStructSize = sizeof(urlComp);

    urlComp.lpszHostName = host_wc;
    urlComp.dwHostNameLength  = (DWORD)(url_len + 1);

    urlComp.lpszUrlPath = path_wc;
    urlComp.dwUrlPathLength   = (DWORD)(url_len + 1);

    // Crack the url_wc.
    if (!WinHttpCrackUrl(url_wc, 0, 0, &urlComp)) {
        LogMsg("Error in WinHttpCrackUrl: %lu", GetLastError());
        goto error_out;
    }

    char buffer[16 * 1024];

    // Use WinHttpOpen to obtain a session handle.
    hSession = WinHttpOpen( L"sbh",
            WINHTTP_ACCESS_TYPE_DEFAULT_PROXY,
            WINHTTP_NO_PROXY_NAME,
            WINHTTP_NO_PROXY_BYPASS, 0 );

    if (NULL == hSession) {
        LogMsg("Can't open HTTP session");
        goto error_out;
    }

    timeout *= 1000;
    if (! WinHttpSetTimeouts(hSession, timeout, timeout, timeout, timeout)) {
        LogMsg("can't set timeouts");
        goto error_out;
    }

    hConnect = WinHttpConnect(hSession, host_wc, urlComp.nPort, 0);
    if (NULL == hConnect) {
        LogMsg("Can't open HTTP session");
        goto error_out;
    }

    hRequest = WinHttpOpenRequest(hConnect, L"GET", path_wc, NULL, WINHTTP_NO_REFERER,
                                  WINHTTP_DEFAULT_ACCEPT_TYPES,
                                  (urlComp.nScheme == INTERNET_SCHEME_HTTPS) ? WINHTTP_FLAG_SECURE : 0);
    if (NULL == hRequest) {
        LogMsg("Can't open HTTP request: %lu", GetLastError());
        goto error_out;
    }

    bResults = WinHttpSendRequest(hRequest, WINHTTP_NO_ADDITIONAL_HEADERS, 0,
                                  WINHTTP_NO_REQUEST_DATA, 0, 0, 0);
    if (! bResults) {
        LogMsg("Can't send HTTP request: %lu", GetLastError());
        goto error_out;
    }

    bResults = WinHttpReceiveResponse(hRequest, NULL);
    if (! bResults) {
        LogMsg("Can't receive response: %lu", GetLastError());
        goto error_out;
    }

    while (1) {
        DWORD res = WinHttpQueryDataAvailable(hRequest, &dwSize);
        if (!res) {
            LogMsg("%lu, Error %lu in WinHttpQueryDataAvailable.", res, GetLastError());
            goto error_out;
        }

        // LogMsg("dwSize %d", dwSize);
        if (0 == dwSize) {
            break;
        }

        while (dwSize > 0) {
            int get_len = (dwSize < sizeof(buffer) ? dwSize : sizeof(buffer));

            bResults = WinHttpReadData(hRequest, buffer, get_len, &dwDownloaded);
            if (! bResults){
               LogMsg("Error %lu in WinHttpReadData.", GetLastError());
               goto error_out;
            }

            data.append(buffer, dwDownloaded);
            dwSize -= dwDownloaded;
        }
    }

    result = true;

error_out:
    // Close any open handles.
    if (hRequest) WinHttpCloseHandle(hRequest);
    if (hConnect) WinHttpCloseHandle(hConnect);
    if (hSession) WinHttpCloseHandle(hSession);

    //LogMsg("HttpGet result: %d", result);
    return result;
}

#else   // Linux or MacOS
#include <curl/curl.h>
static size_t
write_cb(const void *ptr, size_t size, size_t nmemb, void *userdata)
{
    auto len = size * nmemb;
    std::string *data = static_cast<std::string *>(userdata);
    data->append((const char *)ptr, len);
    return len;
}

bool
HttpGet(const std::string& url, std::string& data, int timeout)
{
    CURL *curl;
    CURLcode res;
    curl_global_init(CURL_GLOBAL_ALL);
    curl = curl_easy_init();
    if(curl == NULL)
        return 0;

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, timeout);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_cb);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void *)&data);

    curl_easy_setopt(curl, CURLOPT_HTTPGET, 1L);
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
    res = curl_easy_perform(curl);

    // Check for errors
    if(res != CURLE_OK) {
        LogMsg("curl_easy_perform() failed: %s\n", curl_easy_strerror(res));
        curl_easy_cleanup(curl);
        curl_global_cleanup();
        return false;
    }

    curl_off_t dl_size;
    res = curl_easy_getinfo(curl, CURLINFO_SIZE_DOWNLOAD_T , &dl_size);
    if(res == CURLE_OK)
        LogMsg("Downloaded %d bytes", (int)dl_size);

    curl_easy_cleanup(curl);
    curl_global_cleanup();
    return true;
}
#endif
